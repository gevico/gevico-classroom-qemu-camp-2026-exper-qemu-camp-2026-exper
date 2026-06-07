// SPDX-License-Identifier: GPL-2.0-or-later

mod bindings;

use core::ffi::c_void;
use std::ffi::c_char;

use hwcore::bindings::DeviceState;

use bindings::{at24c_eeprom_init, i2c_end_transfer, i2c_init_bus, i2c_recv, i2c_send, i2c_start_transfer, I2CBus};

const REG_CTRL: u64 = 0x00;
const REG_STATUS: u64 = 0x04;
const REG_ADDR: u64 = 0x08;
const REG_DATA: u64 = 0x0c;
const REG_PRESCALE: u64 = 0x10;

const CTRL_EN: u32 = 1 << 0;
const CTRL_START: u32 = 1 << 1;
const CTRL_STOP: u32 = 1 << 2;
const CTRL_RW: u32 = 1 << 3;

const ST_BUSY: u32 = 1 << 0;
const ST_ACK: u32 = 1 << 1;
const ST_DONE: u32 = 1 << 2;

const AT24C02_ADDR: u8 = 0x50;
const AT24C02_SIZE: u32 = 256;

const RSPI_REG_CR1: u64 = 0x00;
const RSPI_REG_SR: u64 = 0x04;
const RSPI_REG_DR: u64 = 0x08;
const RSPI_REG_CS: u64 = 0x0c;

const RSPI_CR1_SPE: u32 = 1 << 0;
const RSPI_SR_RXNE: u32 = 1 << 0;
const RSPI_SR_TXE: u32 = 1 << 1;
const RSPI_SR_OVERRUN: u32 = 1 << 4;

const AT25_CMD_WREN: u8 = 0x06;
const AT25_CMD_RDSR: u8 = 0x05;
const AT25_CMD_READ: u8 = 0x03;
const AT25_CMD_WRITE: u8 = 0x02;

const AT25_SR_WIP: u8 = 1 << 0;
const AT25_SR_WEL: u8 = 1 << 1;

const AT25_SIZE: usize = 256;

const BUS_NAME: &[u8] = b"i2c-gpio-bus\0";

#[repr(C)]
struct I2CGpioRust {
    ctrl: u32,
    status: u32,
    addr: u8,
    data: u8,
    prescale: u32,
    started: bool,
    bus: *mut I2CBus,
}

#[derive(Copy, Clone)]
enum SpiPhase {
    Idle,
    StatusRead,
    ReadAddr,
    ReadData,
    WriteAddr,
    WriteData,
}

#[repr(C)]
struct SPIGpioRust {
    cr1: u32,
    sr: u32,
    dr: u8,
    cs: u32,
    phase: SpiPhase,
    read_addr: u8,
    write_addr: u8,
    write_started: bool,
    wel: bool,
    wip: bool,
    flash: [u8; AT25_SIZE],
}

impl I2CGpioRust {
    fn set_done_ack(&mut self, ack: bool) {
        self.status &= !(ST_ACK | ST_DONE);
        if ack {
            self.status |= ST_ACK;
        }
        self.status |= ST_DONE;
    }

    fn do_ctrl(&mut self, val: u32) {
        self.ctrl = val;

        if (val & CTRL_EN) == 0 {
            self.status &= !(ST_BUSY | ST_ACK);
            self.status |= ST_DONE;
            return;
        }

        if (val & CTRL_START) != 0 {
            let is_recv = (val & CTRL_RW) != 0;
            let rc = unsafe { i2c_start_transfer(self.bus, self.addr, is_recv) };

            self.started = rc == 0;
            if self.started {
                self.status |= ST_BUSY;
            } else {
                self.status &= !ST_BUSY;
            }
            self.set_done_ack(rc == 0);
            return;
        }

        if (val & CTRL_STOP) != 0 {
            if self.started {
                unsafe {
                    i2c_end_transfer(self.bus);
                }
            }
            self.started = false;
            self.status &= !(ST_BUSY | ST_ACK);
            self.status |= ST_DONE;
            return;
        }

        if !self.started {
            self.set_done_ack(false);
            return;
        }

        if (val & CTRL_RW) != 0 {
            self.data = unsafe { i2c_recv(self.bus) };
            self.set_done_ack(true);
        } else {
            let rc = unsafe { i2c_send(self.bus, self.data) };
            self.set_done_ack(rc == 0);
        }
    }
}

impl SPIGpioRust {
    fn is_enabled(&self) -> bool {
        (self.cr1 & RSPI_CR1_SPE) != 0
    }

    fn status_byte(&self) -> u8 {
        let mut sr = 0u8;
        if self.wip {
            sr |= AT25_SR_WIP;
        }
        if self.wel {
            sr |= AT25_SR_WEL;
        }
        sr
    }

    fn looks_like_cmd(tx: u8) -> bool {
        matches!(tx, AT25_CMD_WREN | AT25_CMD_RDSR | AT25_CMD_READ | AT25_CMD_WRITE)
    }

    fn exec_cmd(&mut self, tx: u8) -> u8 {
        match tx {
            AT25_CMD_WREN => {
                self.wel = true;
                self.phase = SpiPhase::Idle;
                0
            }
            AT25_CMD_RDSR => {
                self.phase = SpiPhase::StatusRead;
                0
            }
            AT25_CMD_READ => {
                self.phase = SpiPhase::ReadAddr;
                0
            }
            AT25_CMD_WRITE => {
                if self.wel {
                    self.phase = SpiPhase::WriteAddr;
                    self.write_started = false;
                } else {
                    self.phase = SpiPhase::Idle;
                }
                0
            }
            _ => {
                self.phase = SpiPhase::Idle;
                0
            }
        }
    }

    fn xfer(&mut self, tx: u8) -> u8 {
        match self.phase {
            SpiPhase::Idle => self.exec_cmd(tx),
            SpiPhase::StatusRead => {
                let sr = self.status_byte();
                self.phase = SpiPhase::Idle;
                sr
            }
            SpiPhase::ReadAddr => {
                self.read_addr = tx;
                self.phase = SpiPhase::ReadData;
                0
            }
            SpiPhase::ReadData => {
                let ret = self.flash[self.read_addr as usize];
                self.read_addr = self.read_addr.wrapping_add(1);
                ret
            }
            SpiPhase::WriteAddr => {
                self.write_addr = tx;
                self.write_started = false;
                self.phase = SpiPhase::WriteData;
                0
            }
            SpiPhase::WriteData => {
                if self.write_started && Self::looks_like_cmd(tx) {
                    self.wip = false;
                    self.phase = SpiPhase::Idle;
                    return self.exec_cmd(tx);
                }

                self.wip = true;
                self.flash[self.write_addr as usize] = tx;
                self.write_addr = self.write_addr.wrapping_add(1);
                self.write_started = true;
                self.wip = false;
                0
            }
        }
    }
}

#[no_mangle]
pub unsafe extern "C" fn i2c_gpio_rust_new(parent: *mut DeviceState) -> *mut c_void {
    let bus = unsafe { i2c_init_bus(parent, BUS_NAME.as_ptr() as *const c_char) };
    if bus.is_null() {
        return core::ptr::null_mut();
    }

    unsafe {
        at24c_eeprom_init(bus, AT24C02_ADDR, AT24C02_SIZE);
    }

    let state = Box::new(I2CGpioRust {
        ctrl: 0,
        status: 0,
        addr: 0,
        data: 0,
        prescale: 0,
        started: false,
        bus,
    });

    Box::into_raw(state) as *mut c_void
}

#[no_mangle]
pub unsafe extern "C" fn i2c_gpio_rust_free(state: *mut c_void) {
    if state.is_null() {
        return;
    }
    let mut boxed = unsafe { Box::from_raw(state as *mut I2CGpioRust) };
    if boxed.started {
        unsafe {
            i2c_end_transfer(boxed.bus);
        }
    }
    boxed.started = false;
}

#[no_mangle]
pub unsafe extern "C" fn i2c_gpio_rust_reset(state: *mut c_void) {
    if state.is_null() {
        return;
    }
    let s = unsafe { &mut *(state as *mut I2CGpioRust) };
    if s.started {
        unsafe {
            i2c_end_transfer(s.bus);
        }
    }
    s.ctrl = 0;
    s.status = 0;
    s.addr = 0;
    s.data = 0;
    s.prescale = 0;
    s.started = false;
}

#[no_mangle]
pub unsafe extern "C" fn i2c_gpio_rust_read(state: *mut c_void, offset: u64) -> u32 {
    if state.is_null() {
        return 0;
    }
    let s = unsafe { &mut *(state as *mut I2CGpioRust) };
    match offset {
        REG_CTRL => s.ctrl,
        REG_STATUS => s.status,
        REG_ADDR => u32::from(s.addr),
        REG_DATA => u32::from(s.data),
        REG_PRESCALE => s.prescale,
        _ => 0,
    }
}

#[no_mangle]
pub unsafe extern "C" fn i2c_gpio_rust_write(state: *mut c_void, offset: u64, value: u32) {
    if state.is_null() {
        return;
    }
    let s = unsafe { &mut *(state as *mut I2CGpioRust) };
    match offset {
        REG_CTRL => s.do_ctrl(value),
        REG_ADDR => s.addr = value as u8,
        REG_DATA => s.data = value as u8,
        REG_PRESCALE => s.prescale = value,
        _ => {}
    }
}

#[no_mangle]
pub unsafe extern "C" fn spi_gpio_rust_new(_parent: *mut DeviceState) -> *mut c_void {
    let state = Box::new(SPIGpioRust {
        cr1: 0,
        sr: 0,
        dr: 0,
        cs: 0,
        phase: SpiPhase::Idle,
        read_addr: 0,
        write_addr: 0,
        write_started: false,
        wel: false,
        wip: false,
        flash: [0; AT25_SIZE],
    });

    Box::into_raw(state) as *mut c_void
}

#[no_mangle]
pub unsafe extern "C" fn spi_gpio_rust_free(state: *mut c_void) {
    if state.is_null() {
        return;
    }
    let _boxed = unsafe { Box::from_raw(state as *mut SPIGpioRust) };
}

#[no_mangle]
pub unsafe extern "C" fn spi_gpio_rust_reset(state: *mut c_void) {
    if state.is_null() {
        return;
    }
    let s = unsafe { &mut *(state as *mut SPIGpioRust) };
    s.cr1 = 0;
    s.sr = 0;
    s.dr = 0;
    s.cs = 0;
    s.phase = SpiPhase::Idle;
    s.read_addr = 0;
    s.write_addr = 0;
    s.write_started = false;
    s.wel = false;
    s.wip = false;
    s.flash.fill(0);
}

#[no_mangle]
pub unsafe extern "C" fn spi_gpio_rust_read(state: *mut c_void, offset: u64) -> u32 {
    if state.is_null() {
        return 0;
    }
    let s = unsafe { &mut *(state as *mut SPIGpioRust) };
    match offset {
        RSPI_REG_CR1 => s.cr1,
        RSPI_REG_SR => s.sr,
        RSPI_REG_DR => {
            s.sr &= !RSPI_SR_RXNE;
            u32::from(s.dr)
        }
        RSPI_REG_CS => s.cs,
        _ => 0,
    }
}

#[no_mangle]
pub unsafe extern "C" fn spi_gpio_rust_write(state: *mut c_void, offset: u64, value: u32) {
    if state.is_null() {
        return;
    }
    let s = unsafe { &mut *(state as *mut SPIGpioRust) };
    match offset {
        RSPI_REG_CR1 => {
            s.cr1 = value;
            if s.is_enabled() {
                s.sr |= RSPI_SR_TXE;
            } else {
                s.sr &= !(RSPI_SR_TXE | RSPI_SR_RXNE | RSPI_SR_OVERRUN);
                s.phase = SpiPhase::Idle;
            }
        }
        RSPI_REG_DR => {
            if !s.is_enabled() {
                return;
            }
            let rx = s.xfer(value as u8);
            s.dr = rx;
            s.sr |= RSPI_SR_TXE | RSPI_SR_RXNE;
            s.sr &= !RSPI_SR_OVERRUN;
        }
        RSPI_REG_CS => {
            s.cs = value;
            s.phase = SpiPhase::Idle;
            s.write_started = false;
        }
        _ => {}
    }
}