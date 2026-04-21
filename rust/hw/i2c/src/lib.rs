// Copyright 2025 HUST OpenAtom Open Source Club.
// Author(s): Chen Miao <chenmiao@openatom.club>
// Author(s): Chao Liu <chao.liu@openatom.club>
// SPDX-License-Identifier: GPL-2.0-or-later

//! I2C bus model for G233 SoC (QEMU Camp 2026 Rust experiment).
//!
//! This module provides a pure-Rust I2C bus abstraction modeled after
//! the upstream QEMU C I2C infrastructure (`include/hw/i2c/i2c.h`).
//!
//! Students must implement the TODO-marked methods to make the unit
//! tests pass. The design follows the upstream pattern:
//!
//! - [`I2CEvent`]: bus state change events (START_RECV, START_SEND, FINISH, NACK)
//! - [`I2CSlave`]: trait for I2C slave devices (send, recv, event)
//! - [`I2CBus`]: bus that manages slave devices and routes transfers

pub mod bus;

// ─── I2C Event ───────────────────────────────────────────────────────

/// I2C bus events, mirroring `enum i2c_event` from upstream C code.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum I2CEvent {
    /// Master requests to read from slave
    StartRecv,
    /// Master requests to write to slave
    StartSend,
    /// Transfer finished
    Finish,
    /// Master NACKed a received byte
    Nack,
}

// ─── I2C Slave trait ─────────────────────────────────────────────────

/// Trait representing an I2C slave device on the bus.
///
/// This mirrors `I2CSlaveClass` from upstream QEMU:
/// - `address()`: the 7-bit slave address
/// - `event()`: notification of bus state changes (START/FINISH/NACK)
/// - `send()`: master-to-slave data byte, returns 0 for ACK, non-zero for NACK
/// - `recv()`: slave-to-master data byte
pub trait I2CSlave {
    /// Return the 7-bit I2C address of this device.
    fn address(&self) -> u8;

    /// Notify the slave of a bus state change.
    ///
    /// For `StartRecv`/`StartSend`, return 0 to ACK or non-zero to NACK.
    /// For `Finish`/`Nack`, the return value is ignored.
    fn event(&mut self, event: I2CEvent) -> i32 {
        let _ = event;
        0
    }

    /// Master sends a data byte to this slave.
    /// Returns 0 for ACK, non-zero for NACK.
    fn send(&mut self, data: u8) -> i32;

    /// Slave returns a data byte to the master.
    fn recv(&mut self) -> u8;
}

// ─── I2C Bus ─────────────────────────────────────────────────────────

/// A simple I2C bus that manages a list of slave devices.
///
/// Mirrors `I2CBus` from upstream QEMU. The bus holds references to
/// attached slaves and routes transfers based on 7-bit address matching.
#[allow(dead_code)]
pub struct I2CBus {
    devices: Vec<Box<dyn I2CSlave>>,
    /// Address of the currently selected slave (set by `start_transfer`)
    current_addr: Option<u8>,
    /// Transfer direction: true = recv (slave→master), false = send (master→slave)
    is_recv: bool,
}

impl I2CBus {
    /// Create an empty bus with no attached devices.
    pub fn new() -> Self {
        Self {
            devices: Vec::new(),
            current_addr: None,
            is_recv: false,
        }
    }

    /// Attach a slave device to the bus.
    pub fn attach(&mut self, _device: Box<dyn I2CSlave>) {
        // TODO: push the device onto the bus
        self.devices.push(_device);
    }

    /// Return the number of devices on the bus.
    pub fn device_count(&self) -> usize {
        // TODO: return actual count
        self.devices.len()
    }

    /// Check if the bus is busy (a transfer is in progress).
    pub fn is_busy(&self) -> bool {
        self.current_addr.is_some()
    }

    /// Start a transfer to the slave at `address`.
    ///
    /// If `is_recv` is true, the master wants to read (START_RECV).
    /// If `is_recv` is false, the master wants to write (START_SEND).
    ///
    /// Returns 0 on success (slave ACKed), -1 if no slave responds (NACK).
    ///
    /// This mirrors `i2c_start_transfer()` from upstream.
    pub fn start_transfer(&mut self, _address: u8, _is_recv: bool) -> i32 {
        // TODO: find a device matching _address, call its event()
        // with StartRecv or StartSend. If ACKed, store current_addr
        // and is_recv. Return 0 on ACK, -1 on NACK.
        
        for slave in &mut self.devices {
            if slave.address() == _address {
                if _is_recv {
                    match slave.event(I2CEvent::StartRecv) {
                        0 => {
                            self.current_addr = Some(_address);
                            self.is_recv = _is_recv;
                            return 0;
                        },
                        _ =>return -1,
                    };
                } else {
                    match slave.event(I2CEvent::StartSend) {
                        0 => {
                            self.current_addr = Some(_address);
                            self.is_recv = _is_recv;
                            return 0;
                        },
                        _ =>return -1,
                    };
                }
            }
        }
        -1
    }

    /// End the current transfer, sending Finish event to the active slave.
    ///
    /// Mirrors `i2c_end_transfer()` from upstream.
    pub fn end_transfer(&mut self) {
        // TODO: send Finish event to the current slave, clear current_addr
        let current_addr:u8;
        if let Some(addr) = self.current_addr {
            current_addr = addr;
        } else {
            return;
        }
        for slave in &mut self.devices {
            if current_addr == slave.address() {
                slave.event(I2CEvent::Finish);
                break;
            }
        }
        self.current_addr = None;
    }

    /// Send a data byte from master to the current slave.
    ///
    /// Returns 0 for ACK, non-zero for NACK.
    /// Mirrors `i2c_send()` from upstream.
    pub fn send(&mut self, _data: u8) -> i32 {
        // TODO: call send() on the current slave
        let current_addr:u8 ;
        if let Some(addr) = self.current_addr {
            current_addr = addr;
        } else {
            return -1;
        }
        for slave in &mut self.devices {
            if slave.address() == current_addr {
                match slave.send(_data) {
                    0 => return 0,
                    _ => return -1,
                };
            }
        }
        0
    }

    /// Receive a data byte from the current slave to master.
    ///
    /// Mirrors `i2c_recv()` from upstream.
    pub fn recv(&mut self) -> u8 {
        // TODO: call recv() on the current slave
        let current_addr:u8 ;
        if let Some(addr) = self.current_addr {
            current_addr = addr;
        } else {
            return 0xFF;
        }
        for slave in &mut self.devices {
            if slave.address() == current_addr {
                return slave.recv();
            }
        }
        0xFF
    }

    // ── Convenience helpers (used by unit tests) ──

    /// Perform a complete write transfer: START_SEND + send bytes + FINISH.
    ///
    /// Returns true on ACK, false on NACK.
    pub fn transfer_write(&mut self, addr: u8, data: &[u8]) -> bool {
        if self.start_transfer(addr, false) != 0 {
            return false;
        }
        for &byte in data {
            if self.send(byte) != 0 {
                self.end_transfer();
                return false;
            }
        }
        self.end_transfer();
        true
    }

    /// Perform a complete read transfer: START_RECV + recv `len` bytes + FINISH.
    ///
    /// Returns None on NACK, Some(bytes) on success.
    pub fn transfer_read(&mut self, addr: u8, len: usize) -> Option<Vec<u8>> {
        if self.start_transfer(addr, true) != 0 {
            return None;
        }
        let mut result = Vec::with_capacity(len);
        for _ in 0..len {
            result.push(self.recv());
        }
        self.end_transfer();
        Some(result)
    }
}

// ─── Unit tests ──────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    /// Mock EEPROM device: 256-byte register file, supports I2C protocol.
    /// Write: first byte = register address, subsequent bytes = data.
    /// Read: returns bytes starting from the last set register address.
    struct MockEeprom {
        addr: u8,
        regs: [u8; 256],
        pointer: u8,
        first_byte: bool,
    }

    impl MockEeprom {
        fn new(addr: u8) -> Self {
            Self {
                addr,
                regs: [0xFF; 256],
                pointer: 0,
                first_byte: true,
            }
        }
    }

    impl I2CSlave for MockEeprom {
        fn address(&self) -> u8 {
            self.addr
        }

        fn event(&mut self, event: I2CEvent) -> i32 {
            if event == I2CEvent::StartSend {
                self.first_byte = true;
            }
            0
        }

        fn send(&mut self, data: u8) -> i32 {
            if self.first_byte {
                self.pointer = data;
                self.first_byte = false;
            } else {
                self.regs[self.pointer as usize] = data;
                self.pointer = self.pointer.wrapping_add(1);
            }
            0
        }

        fn recv(&mut self) -> u8 {
            let val = self.regs[self.pointer as usize];
            self.pointer = self.pointer.wrapping_add(1);
            val
        }
    }

    #[test]
    fn test_i2c_bus_create() {
        let mut bus = I2CBus::new();
        bus.attach(Box::new(MockEeprom::new(0x50)));
        assert_eq!(bus.device_count(), 1, "bus should have 1 device");
        bus.attach(Box::new(MockEeprom::new(0x51)));
        assert_eq!(bus.device_count(), 2, "bus should have 2 devices");
    }

    #[test]
    fn test_i2c_bus_read_write() {
        let mut bus = I2CBus::new();
        bus.attach(Box::new(MockEeprom::new(0x50)));

        // Write: addr_byte=0x10, data=0xDE
        let ack = bus.transfer_write(0x50, &[0x10, 0xDE]);
        assert!(ack, "write to attached device must ACK");

        // Set read pointer: write register address first
        bus.transfer_write(0x50, &[0x10]);

        // Read back 1 byte
        let data = bus.transfer_read(0x50, 1);
        assert_eq!(data, Some(vec![0xDE]), "read back must match written value");
    }

    #[test]
    fn test_i2c_bus_nack() {
        let mut bus = I2CBus::new();
        bus.attach(Box::new(MockEeprom::new(0x50)));

        // Read from non-existent address
        let data = bus.transfer_read(0x77, 1);
        assert_eq!(data, None, "read from non-existent address must NACK");

        // Write to non-existent address
        let ack = bus.transfer_write(0x77, &[0x00]);
        assert!(!ack, "write to non-existent address must NACK");
    }
}
