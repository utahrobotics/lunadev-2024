use unros::anyhow;
use unros::log::error;

use crate::{actuators::Arms, drive::Drive};
use std::fs::{self, DirEntry};
use std::process::Command;

pub fn connect_to_serial() -> anyhow::Result<(Option<Arms>, Option<Drive>)> {
    let dev_directory = fs::read_dir("/dev")?;
    let acm_devices: Vec<DirEntry> = dev_directory
        .filter_map(|entry| {
            let entry = entry.ok()?;
            if entry.file_name().as_encoded_bytes().starts_with(b"ttyACM") {
                Some(entry)
            } else {
                None
            }
        })
        .collect();

    if acm_devices.is_empty() {
        return Err(anyhow::anyhow!("No ttyACM devices found"));
    }
    if acm_devices.len() != 4 {
        return Err(anyhow::anyhow!(
            "Expected 4 ttyACM devices, got {}",
            acm_devices.len()
        ));
    }

    if acm_devices.is_empty() {
        return Err(anyhow::anyhow!("No ttyACM devices found"));
    }

    let mut micro_python_paths = Vec::with_capacity(2);
    let mut vesc_paths = Vec::with_capacity(2);
    for entry in &acm_devices {
        let path = entry.path();
        let output = Command::new("udevadm")
            .arg("info")
            .arg("--query=path")
            .arg("--name")
            .arg(&path)
            .arg("--query=all")
            .output()?;
        let stdout = String::from_utf8_lossy(&output.stdout);
        if stdout.contains("MicroPython") {
            if micro_python_paths.len() >= 2 {
                return Err(anyhow::anyhow!("More than two MicroPython devices found"));
            }
            micro_python_paths.push(path);
        } else if vesc_paths.len() >= 2 {
            return Err(anyhow::anyhow!("More than two VESC devices found"));
        } else {
            vesc_paths.push(path);
        }
    }

    let mut arms = None;

    if micro_python_paths.len() < 2 {
        arms = Some(Arms::new(
            micro_python_paths[0].to_string_lossy(),
            micro_python_paths[1].to_string_lossy(),
        ));
    }

    let mut drive = None;

    if vesc_paths.len() < 2 {
        match Drive::new(
            vesc_paths[0].to_string_lossy(),
            vesc_paths[1].to_string_lossy(),
        ) {
            Ok(d) => drive = Some(d),
            Err(e) => error!("{e}"),
        }
    }

    if arms.is_none() {
        error!("Failed to connect to arms");
    }

    if drive.is_none() {
        error!("Failed to connect to drive");
    }

    Ok((arms, drive))
}
