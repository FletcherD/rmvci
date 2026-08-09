//! Response validation shared by the KWP2000 and UDS clients — the positive/
//! negative framing is identical between the two protocols.

use crate::error::{Error, Result};

/// A positive response echoes `service | 0x40` (ISO 14229/14230 framing).
pub(crate) const POSITIVE_RESPONSE_OFFSET: u8 = 0x40;
/// A negative response begins with this service id, then `<service> <nrc>`.
pub(crate) const NEGATIVE_RESPONSE_SID: u8 = 0x7f;

/// Validate a positive response or classify a negative one. Returns the bytes
/// after the echoed service id.
pub(crate) fn parse_positive(sid: u8, resp: Vec<u8>) -> Result<Vec<u8>> {
    let positive = sid.wrapping_add(POSITIVE_RESPONSE_OFFSET);
    match resp.split_first() {
        None => Err(Error::Empty),
        Some((&NEGATIVE_RESPONSE_SID, tail)) => Err(Error::Negative {
            service: tail.first().copied().unwrap_or(sid),
            nrc: tail.get(1).copied().unwrap_or(0),
        }),
        Some((&first, tail)) if first == positive => Ok(tail.to_vec()),
        Some(_) => Err(Error::Unexpected { expected: positive, got: resp }),
    }
}

/// Strip an echoed identifier prefix (the LID/DID) from a positive-response
/// payload, returning just the data bytes.
pub(crate) fn strip_echo(resp: &[u8], echo: &[u8]) -> Result<Vec<u8>> {
    if resp.len() >= echo.len() && &resp[..echo.len()] == echo {
        Ok(resp[echo.len()..].to_vec())
    } else {
        Err(Error::Unexpected {
            expected: echo.first().map_or(0, |b| b | POSITIVE_RESPONSE_OFFSET),
            got: resp.to_vec(),
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn positive_strips_sid() {
        assert_eq!(parse_positive(0x21, vec![0x61, 0x0d, 0x09]).unwrap(), vec![0x0d, 0x09]);
        assert_eq!(parse_positive(0x22, vec![0x62, 0x01, 0x43, 0x7b]).unwrap(), vec![0x01, 0x43, 0x7b]);
    }

    #[test]
    fn negative_is_classified() {
        let e = parse_positive(0x21, vec![0x7f, 0x21, 0x31]).unwrap_err();
        assert!(matches!(e, Error::Negative { service: 0x21, nrc: 0x31 }));
        assert!(e.is_request_out_of_range());
    }

    #[test]
    fn empty_and_unexpected() {
        assert!(matches!(parse_positive(0x21, vec![]).unwrap_err(), Error::Empty));
        assert!(matches!(
            parse_positive(0x21, vec![0x50]).unwrap_err(),
            Error::Unexpected { expected: 0x61, .. }
        ));
    }

    #[test]
    fn echo_stripping() {
        assert_eq!(strip_echo(&[0x0d, 0x09], &[0x0d]).unwrap(), vec![0x09]);
        assert_eq!(strip_echo(&[0xf1, 0x90, 0xaa], &[0xf1, 0x90]).unwrap(), vec![0xaa]);
        assert!(strip_echo(&[0x0e, 0x09], &[0x0d]).is_err());
    }
}
