#[cfg(unix)]
fn main() {
    use std::process::Command;

    use sudo::{escalate_if_needed, RunningAs};

    match sudo::check() {
        sudo::RunningAs::Root => {}
        sudo::RunningAs::User => {
            eprintln!("This program must be run as root.");
            return;
        }
        sudo::RunningAs::Suid => match escalate_if_needed() {
            Ok(RunningAs::Root) => {}
            _ => {
                eprintln!("This program must be run as root.");
                return;
            }
        }
    }

    println!("Plug in the device now");

    loop {
        let output = Command::new("dmesg")
            .output()
            .expect("Failed to execute dmesg");

        let stdout = String::from_utf8(output.stdout).expect("Failed to read stdout");
        for line in stdout.split('\n') {
            if line.contains("usb") {
                println!("{}", line);
            }
        }

        std::thread::sleep(std::time::Duration::from_secs(1));
    }
}

#[cfg(not(unix))]
fn main() {
    eprintln!("This program is only supported on Unix systems.");
}
