/// A implied abstract class that all commands must implement. Each command must define the four methods below to execute
/// their functionality
pub trait Command {
    fn start(&self) {}
    fn run_every_cycle(&self) {}
    fn has_finished(&self) -> bool { return false; }
    fn finish(&self) {}
}