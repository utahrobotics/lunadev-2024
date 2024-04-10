use unros::anyhow;

use std::os::unix::ffi::OsStrExt;
use crate::{actuators::Arms, drive::Drive};
use std::fs::{self, DirEntry};
use std::process::{Command, Stdio};

pub fn connect_to_serial() -> anyhow::Result<(Arms, Drive)> {
    let dev_directory = fs::read_dir("/dev")?;
    let acm_devices: Vec<DirEntry> = dev_directory
        .filter_map(|entry| {
            let entry = entry.ok()?;
            if entry
                .file_name()
                .as_bytes()
                .starts_with(b"ttyACM")
            {
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

    if acm_devices.len() != 4 {
        return Err(anyhow::anyhow!("Expected 4 ttyACM devices, got {}", acm_devices.len()));
    }

    let mut micro_python_paths = [None, None];
    let mut vesc_paths = Vec::with_capacity(2);
    let mut micro_py_count = 0usize;
    for entry in acm_devices.iter().take(2) {
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
            if micro_py_count >= 2 {
                return Err(anyhow::anyhow!("More than two MicroPython devices found"));
            }
            micro_python_paths[micro_py_count] = Some(path);
            micro_py_count += 1;
        } else if vesc_paths.len() >= 2 {
            return Err(anyhow::anyhow!("More than two VESC devices found"));
        } else {
            vesc_paths.push(path);
        }
    }

    if micro_py_count < 2 {
        return Err(anyhow::anyhow!("Expected 2 MicroPython devices, got {}", micro_py_count));
    }

    let micro_python_paths = micro_python_paths.map(|x| x.unwrap());

    todo!()
}
