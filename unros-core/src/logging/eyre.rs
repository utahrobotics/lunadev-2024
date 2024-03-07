use std::fmt;
use std::fmt::Write;

use color_eyre::{
    owo_colors::{style, OwoColorize},
    section::PanicMessage,
};
use log::error;

#[derive(Clone, Copy)]
pub(super) struct UnrosEyreMessage;

// Adapted from the default PanicMessage in color_eyre
impl PanicMessage for UnrosEyreMessage {
    fn display(&self, pi: &std::panic::PanicInfo<'_>, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let mut log = String::new();
        writeln!(
            f,
            "{}",
            "The application panicked (crashed).".style(style().red())
        )?;
        writeln!(log, "The application panicked (crashed).")?;

        // Print panic message.
        let payload = pi
            .payload()
            .downcast_ref::<String>()
            .map(String::as_str)
            .or_else(|| pi.payload().downcast_ref::<&str>().copied())
            .unwrap_or("<non string panic payload>");

        write!(f, "Message:  ")?;
        writeln!(f, "{}", payload.style(style().cyan()))?;
        write!(log, "\tMessage:  ")?;
        writeln!(log, "{payload}")?;

        // If known, print panic location.
        write!(f, "Location: ")?;
        write!(log, "\tLocation: ")?;

        if let Some(loc) = pi.location() {
            write!(f, "{}", loc.file().style(style().purple()))?;
            write!(f, ":")?;
            write!(f, "{}", loc.line().style(style().purple()))?;

            write!(log, "{}", loc.file())?;
            write!(log, ":")?;
            write!(log, "{}", loc.line())?;
        } else {
            write!(f, "<unknown>")?;
            write!(log, "<unknown>")?;
        }

        error!(target: "panic", "{log}");

        Ok(())
    }
}
