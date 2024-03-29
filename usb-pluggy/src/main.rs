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
        },
    }

    let output = Command::new("dmesg")
        .arg("-C")
        .output()
        .expect("Failed to clear dmesg");

    if !output.status.success() {
        eprintln!("Failed to clear dmesg");
        return;
    }

    println!("Plug in the device now");

    loop {
        let output = Command::new("dmesg")
            .output()
            .expect("Failed to execute dmesg");

        if !output.status.success() {
            eprintln!("Failed to execute dmesg");
            return;
        }

        let clear_output = Command::new("dmesg")
            .arg("-C")
            .output()
            .expect("Failed to clear dmesg");

        if !clear_output.status.success() {
            eprintln!("Failed to clear dmesg");
            return;
        }

        let stdout = String::from_utf8(output.stdout).expect("Failed to read stdout");
        let mut lines = vec![];
        for line in stdout.split('\n') {
            if line.contains("usb") && !line.contains("disconnect") {
                lines.push(line);
            }
        }

        if lines.is_empty() {
            std::thread::sleep(std::time::Duration::from_secs(1));
            continue;
        }

        let line = lines[0].split_at(lines[0].find("usb ").unwrap() + 4).1;
        let usb_path = line.split_at(line.find(":").unwrap()).0;
        println!("Identified the following device at {usb_path}:");

        let mut product = None;
        let mut manufacturer = None;
        let mut serial_no = None;
        let mut id_vendor = None;
        let mut id_product = None;

        for line in lines.iter().copied() {
            if line.contains("Product: ") {
                product = Some(line.split_at(line.find("Product: ").unwrap() + 8).1);
            } else if line.contains("Manufacturer: ") {
                manufacturer = Some(line.split_at(line.find("Manufacturer: ").unwrap() + 13).1);
            } else if line.contains("SerialNumber: ") {
                serial_no = Some(line.split_at(line.find("SerialNumber: ").unwrap() + 14).1);
            } else if line.contains("idVendor=") {
                id_vendor = Some(
                    line.split_at(line.find("idVendor=").unwrap() + 9)
                        .1
                        .split_at(4)
                        .0,
                );
                if line.contains("idProduct=") {
                    id_product = Some(
                        line.split_at(line.find("idProduct=").unwrap() + 10)
                            .1
                            .split_at(4)
                            .0,
                    );
                }
            }
        }

        if let Some(product) = product {
            println!("Product: {product}");
        } else {
            eprintln!("Failed to identify product");
        }
        if let Some(manufacturer) = manufacturer {
            println!("Manufacturer: {manufacturer}");
        } else {
            eprintln!("Failed to identify manufacturer");
        }
        if let Some(serial_no) = serial_no {
            println!("Serial Number: {serial_no}");
        } else {
            eprintln!("Failed to identify serial number");
        }
        if let Some(id_product) = id_product {
            println!("Product Id: {id_product}");
        } else {
            eprintln!("Failed to identify product id");
        }
        if let Some(id_vendor) = id_vendor {
            println!("Vendor Id: {id_vendor}");
        } else {
            eprintln!("Failed to identify vendor id");
        }
        println!();
        println!("Would you like to generate a udev rule for this? (y/n)");

        let mut input = String::new();
        std::io::stdin()
            .read_line(&mut input)
            .expect("Failed to read input");
        input = input.to_ascii_lowercase();
        let ans = input.trim();

        if ans == "y" || ans == "yes" {
            println!("What would you like the symlink to be named?");
            input.clear();
            std::io::stdin()
                .read_line(&mut input)
                .expect("Failed to read input");
            let name = input.trim();

            let mut rule = format!("SUBSYSTEM==\"usb\", KERNEL==\"{usb_path}\", ");
            if let Some(serial_no) = serial_no {
                rule.push_str(&format!("ATTRS{{serial}}==\"{serial_no}\", "));
            }
            if let Some(id_product) = id_product {
                rule.push_str(&format!("ATTRS{{idProduct}}==\"{id_product}\", "));
            }
            if let Some(id_vendor) = id_vendor {
                rule.push_str(&format!("ATTRS{{idVendor}}==\"{id_vendor}\", "));
            }
            rule.push_str(&format!("MODE=\"0666\", SYMLINK+=\"{name}\""));
            println!("\n{rule}");
            println!(
                "This should be added to a file in /etc/udev/rules.d/ with a .rules extension."
            );
            break;
        }
        println!();
    }
}

#[cfg(not(unix))]
fn main() {
    eprintln!("This program is only supported on Unix systems.");
}
