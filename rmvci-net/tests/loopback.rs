//! End-to-end over a real loopback socket: a [`MockTransport`] behind
//! [`serve_connection`] on one thread, a [`TcpTransport`] client on another.
//! Every `Transport` primitive must survive the round trip byte-for-byte.

use std::net::TcpListener;
use std::thread;
use std::time::Duration;

use rmvci_core::transport::mock::Step;
use rmvci_core::transport::{LatencyResult, MockTransport, Transport};
use rmvci_net::{serve_connection, TcpTransport};

#[test]
fn primitives_round_trip_over_tcp() {
    let listener = TcpListener::bind("127.0.0.1:0").unwrap();
    let addr = listener.local_addr().unwrap();

    let server = thread::spawn(move || {
        let (stream, _) = listener.accept().unwrap();
        // One scripted exchange: a write of [0xaa] is answered with 3 bytes,
        // which a later read must drain.
        let mut mock = MockTransport::new([Step::exchange(vec![0xaa], vec![0x11, 0x22, 0x33])]);
        let events = mock.events();
        serve_connection(stream, &mut mock).unwrap();
        // Return the recorded events so the client side can assert on them.
        events.lock().unwrap().clone()
    });

    let mut io = TcpTransport::connect(addr).unwrap();

    // Modem lines forward.
    io.set_modem(true, false).unwrap();

    // Write is asserted byte-exact by the mock (panics through the thread join
    // otherwise), and queues the scripted reply.
    io.write_all(&[0xaa]).unwrap();

    // Read drains the queued reply.
    let mut buf = [0u8; 8];
    let n = io.read(&mut buf, Duration::from_millis(200)).unwrap();
    assert_eq!(&buf[..n], &[0x11, 0x22, 0x33]);

    // Nothing left: an empty read reports a clean timeout, not an error.
    let n = io.read(&mut buf, Duration::from_millis(50)).unwrap();
    assert_eq!(n, 0);

    // The mock has no latency concept; that variant must survive the wire.
    assert_eq!(io.optimize_latency(), LatencyResult::Unavailable);

    // Purge is a plain forwarded call.
    io.purge_rx().unwrap();

    // Disconnect -> the server's read_frame sees EOF and returns cleanly.
    drop(io);

    let events = server.join().unwrap();
    use rmvci_core::transport::mock::Event;
    assert!(events.contains(&Event::Modem { dtr: true, rts: false }));
    assert!(events.contains(&Event::Write(vec![0xaa])));
    assert!(events.contains(&Event::PurgeRx));
}

#[test]
fn version_mismatch_is_rejected() {
    use std::io::{Read, Write};
    let listener = TcpListener::bind("127.0.0.1:0").unwrap();
    let addr = listener.local_addr().unwrap();

    let server = thread::spawn(move || {
        let (stream, _) = listener.accept().unwrap();
        let mut mock = MockTransport::new([]);
        // Expected to Err on the bad hello.
        let _ = serve_connection(stream, &mut mock);
    });

    // Hand-rolled client with a wrong version byte.
    let mut sock = std::net::TcpStream::connect(addr).unwrap();
    sock.write_all(b"RMVN").unwrap();
    sock.write_all(&[0xff]).unwrap();
    sock.flush().unwrap();
    let mut ack = [0u8; 1];
    sock.read_exact(&mut ack).unwrap();
    assert_eq!(ack[0], 0x01, "server must NAK a version mismatch");

    server.join().unwrap();
}
