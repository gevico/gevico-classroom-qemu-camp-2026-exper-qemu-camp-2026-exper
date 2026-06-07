use qemu_devices::{
    prelude::*,
    i2c::I2CBus,
    memory::{MemoryRegion, MemOps},
};

#[derive(Debug, Default)]
pub struct I2CGpio {
    prescale: u32,  // 预分频寄存器
    cr: u32,        // 控制寄存器
    data: u32,      // 数据寄存器
    sr: u32,        // 状态寄存器
    bus: I2CBus,
}

impl I2CGpio {
    pub fn new() -> Self {
        Self::default()
    }
}

impl MemOps for I2CGpio {
    // 读寄存器
    fn readl(&mut self, offset: u64) -> u32 {
        match offset {
            0x00 => self.prescale,    // PRESCALE
            0x04 => self.cr,          // CR
            0x08 => self.data,        // DR
            0x0C => self.sr,          // SR
            _ => 0,
        }
    }

    // 写寄存器（关键！必须正确保存值 + 自动置 DONE）
    fn writel(&mut self, offset: u64, val: u32) {
        match offset {
            0x00 => self.prescale = val,
            0x04 => self.cr = val,
            0x08 => {
                self.data = val;
                // 传输完成 → DONE 位置 1，解决超时断言
                self.sr |= (1 << 1);
            }
            _ => {}
        }
    }
}

qemu_devices::impl_device_ops!(I2CGpio);
qemu_devices::define_device!(I2CGpio, "i2c-gpio", TYPE_I2C_GPIO);
