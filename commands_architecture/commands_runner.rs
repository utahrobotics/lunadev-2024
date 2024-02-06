struct CommandsRunner {
    commands: Vec<Command>,
    current_state: RobotStates,
}

/// In case we want to do different things in different robot modes!
enum RobotStates {
    Autonomous,
    Teleoperated,
    Test,
    Disabled
}

/// Runs all the commands of the robot
impl CommandsRunner {
    pub fn new() -> Self {
        CommandsRunner {
            commands: Vec::new(),
        }
    }

    /// Changes the commands that should be run
    pub fn change_commands(&mut self, new_commands: Vec<Command>) {
        self.commands = new_commands;
    }

    /// Runs all the robot states
    pub fn run(&self) {
        match (self.current_state) {
            // All the robot states should run the commands at this moment besides disabled, which will stop all motors
            RobotStates::Autonomous => self.run_commands(),
            RobotStates::Teleoperated => self.run_commands(),
            RobotStates::Test => self.run_commands(),
        }
    }

    pub fn set_current_state(&mut self, new_state: RobotStates) {
        self.current_state = new_state;

        match (self.current_state) {
            RobotStates::Disabled => {
                // Stop all motors and other processes, so no more commands exist
                self.commands.clear();  // Nothing else should be running on the robot
            }
        }
    }

    /// Runs all the commands of the robot
    pub fn run_commands(&self) {
        if (self.commands.len() == 0) { return 0; }
        let current_command = self.commands[0];

        // Let's run the current command
        if (current_command.has_finished()) {
            current_command.finish();
            self.commands.remove(0);

            // Now let's start the next command
            if (self.commands.len() > 0) {
                self.commands[0].start();
            }
        // If the command has not finished, let's run it
        } else {
            current_command.run_every_cycle();
        }
    }
}