struct Command {}

/// A implied abstract class that all commands must implement. Each command must define the four methods below to execute
/// their functionality
impl Command {
    pub fn new() -> Self {
        Command {}
    }

    pub fn start(&self) {}
    pub fn run_every_cycle(&self) {}
    pub fn has_finished(&self) -> bool { return false; }
    pub fn finish(&self) {}
}