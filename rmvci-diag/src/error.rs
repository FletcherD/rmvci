use automotive_diag::kwp2000::{KwpError, KwpErrorByte};
use automotive_diag::uds::UdsErrorByte;

/// Result alias for diagnostic operations.
pub type Result<T> = std::result::Result<T, Error>;

/// A failed KWP2000 or UDS exchange. The negative-response code table is shared
/// between the two protocols (both derive from the same ISO codes); this type
/// is protocol-neutral and names the NRC via the UDS (superset) table.
#[derive(Debug, thiserror::Error)]
pub enum Error {
    /// The underlying transport (cable / bus) failed.
    #[error(transparent)]
    Transport(#[from] rmvci_core::Error),

    /// The ECU returned a negative response `7F <service> <nrc>`.
    #[error("ECU refused service {service:#04x}: NRC {nrc:#04x} ({:?})", UdsErrorByte::from(*nrc))]
    Negative {
        /// The service id the ECU echoed back.
        service: u8,
        /// The negative response code.
        nrc: u8,
    },

    /// The ECU answered with no bytes at all.
    #[error("empty response from ECU")]
    Empty,

    /// The response was neither the expected positive echo nor a well-formed
    /// negative response.
    #[error("unexpected response: expected positive {expected:#04x}, got {got:02x?}")]
    Unexpected {
        /// The positive-response id we expected (`service | 0x40`).
        expected: u8,
        /// What the ECU actually sent.
        got: Vec<u8>,
    },
}

impl Error {
    /// The negative response code interpreted against the UDS table.
    pub fn uds_nrc(&self) -> Option<UdsErrorByte> {
        match self {
            Error::Negative { nrc, .. } => Some(UdsErrorByte::from(*nrc)),
            _ => None,
        }
    }

    /// The negative response code interpreted against the KWP2000 table.
    pub fn kwp_nrc(&self) -> Option<KwpErrorByte> {
        match self {
            Error::Negative { nrc, .. } => Some(KwpErrorByte::from(*nrc)),
            _ => None,
        }
    }

    /// The raw negative response code byte, if this is a [`Error::Negative`].
    pub fn nrc(&self) -> Option<u8> {
        match self {
            Error::Negative { nrc, .. } => Some(*nrc),
            _ => None,
        }
    }

    /// True if the ECU refused with `requestOutOfRange` — the usual "this ECU
    /// does not serve that identifier" answer, useful when sweeping.
    pub fn is_request_out_of_range(&self) -> bool {
        self.nrc() == Some(KwpError::RequestOutOfRange as u8)
    }
}
