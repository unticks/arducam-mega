#![no_std]

#[cfg(not(any(feature = "3mp", feature = "5mp")))]
compile_error!(
    "at least one of the `arducam-mega/3mp` or `arducam-mega/5mp` features needs to be enabled"
);

use embedded_hal::{delay::DelayUs, spi::SpiBus, spi::SpiDevice};

const READ_REGISTER_MASK: u8 = 0x7f;
const WRITE_REGISTER_MASK: u8 = 0x80;
const SENSOR_STATE_MASK: u8 = 0x03;
const SENSOR_STATE_IDLE: u8 = 0x02;
const SENSOR_RESET_ENABLE: u8 = 0x40;
const SENSOR_ID_MASK: u8 = 0x0f;
const CAPTURE_FINISHED_MASK: u8 = 0x04;
const FIFO_READ_BURST: u8 = 0x3c;
const FIFO_READ_SINGLE: u8 = 0x3d;

#[derive(Debug, Clone)]
enum CameraControl {
    Gain = 0x00, // ISO
    Exposure = 0x01,
    WhiteBalance = 0x02,
}

#[derive(Debug, Clone)]
enum RegisterAddress {
    ArduchipFifo = 0x04,
    SensorReset = 0x07,
    DebugDeviceAddress = 0x0a,
    CaptureFormat = 0x20,
    CaptureResolution = 0x21,
    WhiteBalanceMode = 0x26,
    #[cfg(feature = "5mp")]
    AutoFocus = 0x29,
    GainExposureWhiteBalance = 0x2a,
    SensorId = 0x40,
    SensorState = 0x44,
    FifoSize1 = 0x45,
    FifoSize2 = 0x46,
    FifoSize3 = 0x47,
}

#[derive(Debug, Clone)]
enum ControlValue {
    Disable = 0x00,
    Enable = 0x80,
}

/// The format in which the camera data should be formatted
///
/// You will most likely want to use `Jpeg`, as `Rgb` generates images that are much greater in
/// size than `Jpeg`, and could therefore data blobs that are too big for your MCU to handle.
#[derive(Debug, Clone, Default)]
pub enum CaptureFormat {
    #[default]
    /// Default. Offers a good mix between image quality and data size
    Jpeg = 0x01,
    /// Untested
    Rgb = 0x02,
    /// Untested
    Yuv = 0x03,
}

/// The resolution of the image captured by the camera
///
/// `Qvga` and `Vga` are good test values. `Qqvga` is listed in the SDK, however is also not
/// explicitly added to the camera's supported resolutions. `Qqvga` is therefore listed here but
/// hasn't been tested.
///
/// If both the `3mp` and `5mp` features are enabled, this enum will have a `Fhd` (1080p) default,
/// as that is the highest resolution compatible with both cameras. When only the `3mp` feature is
/// enabled, the enum will default to the camera's maximum supported resolution: `Qxga`. When only
/// the `5mp` feature is enabled, the enum will default to `Wqxga2`.
#[derive(Debug, Clone, Default)]
pub enum CaptureResolution {
    /// QQVGA resolution (160x120). Untested, and the official Arducam SDK does not list this
    /// resolution as supported by either the 3MP or 5MP Arducam Mega.
    Qqvga = 0x00,
    /// QVGA resolution (320x240). Untested.
    Qvga = 0x01,
    /// VGA resolution (640x480). Tested on the Ardumcam Mega 5MP.
    Vga = 0x02,
    /// SVGA resolution (800x600). Untested.
    Svga = 0x03,
    /// HD resolution (1280x720). Untested.
    Hd = 0x04,
    /// SXGAM resolution (1280x960). Untested.
    Sxgam = 0x05,
    /// UXGA resolution (1600x1200). Untested.
    Uxga = 0x06,

    #[cfg_attr(all(feature = "3mp", feature = "5mp"), default)]
    /// FHD resolution (1920x1080). Untested. This is the default choice for this enum when both
    /// the `3mp` and `5mp` features are enabled.
    Fhd = 0x07,

    #[cfg(feature = "3mp")]
    #[cfg_attr(not(feature = "5mp"), default)]
    /// QXGA resolution (2048x1536). Untested. This is the default choice for this enum when the
    /// `3mp` feature (only) is enabled.
    Qxga = 0x08,

    #[cfg(feature = "5mp")]
    #[cfg_attr(not(feature = "3mp"), default)]
    /// WQXGA2 resolution (2592x1944). Untested. This is the default choice for this enum when the
    /// `5mp` feature (only) is enabled.
    Wqxga2 = 0x09,

    /// 96x96 resolution. Untested.
    Res96x96 = 0x0a,
    /// 128x128 resolution. Untested.
    Res128x128 = 0x0b,
    /// 320x320 resolution. Untested.
    Res320x320 = 0x0c,
}

#[derive(Debug, Clone)]
enum ArduchipCommand {
    Clear = 0x01,
    Start = 0x02,
}

/// The white balance mode the camera should use.
///
/// Whether you can use a static value or not will depend on the conditions in which your camera
/// operates. For most consistent results, it is recommended to use a specific mode, which will
/// prevent the camera from randomly switching from one mode to another.
#[derive(Debug, Clone, Default)]
pub enum WhiteBalanceMode {
    #[default]
    Auto = 0x00,
    Sunny = 0x01,
    Office = 0x02,
    Cloudy = 0x03,
    Home = 0x04,
}

/// The main ArducamMega struct.
///
/// This struct is the driver's main entrypoint. By providing it with a configured SPI device and
/// Delay implementation, this driver will be able to configure the Arducam Mega camera, take
/// pictures, and read the picture data back from the camera.
pub struct ArducamMega<SPI, Delay> {
    spi: SPI,
    delay: Delay,
}

impl<SPI, Delay> ArducamMega<SPI, Delay>
where
    SPI: SpiDevice,
    SPI::Bus: SpiBus,
    Delay: DelayUs,
{
    pub fn new(spi: SPI, delay: Delay) -> Self {
        Self { spi, delay }
    }

    fn read_reg(&mut self, addr: RegisterAddress) -> Result<u8, Error<SPI::Error, Delay::Error>> {
        let out: [u8; 1] = [addr as u8 & READ_REGISTER_MASK];
        let mut buf = [0; 3];

        self.spi
            .transfer(&mut buf[..], &out[..])
            .map_err(Error::Spi)?;

        Ok(buf[2])
    }

    fn write_reg(
        &mut self,
        addr: RegisterAddress,
        value: u8,
    ) -> Result<(), Error<SPI::Error, Delay::Error>> {
        let out: [u8; 2] = [addr as u8 | WRITE_REGISTER_MASK, value];
        self.spi.write(&out[..]).map_err(Error::Spi)
    }

    fn wait_idle(&mut self) -> Result<(), Error<SPI::Error, Delay::Error>> {
        while (self.read_reg(RegisterAddress::SensorState)? & SENSOR_STATE_MASK)
            != SENSOR_STATE_IDLE
        {
            self.delay.delay_us(500u32).map_err(Error::Delay)?;
        }

        Ok(())
    }

    pub fn reset(&mut self) -> Result<(), Error<SPI::Error, Delay::Error>> {
        self.write_reg(RegisterAddress::SensorReset, SENSOR_RESET_ENABLE)?;
        self.wait_idle()
    }

    pub fn get_id(&mut self) -> Result<u8, Error<SPI::Error, Delay::Error>> {
        let id = (self.read_reg(RegisterAddress::SensorId)? & SENSOR_ID_MASK) - 1;
        self.wait_idle()?;

        Ok(id)
    }

    /// Sets the auto-focus of the camera.
    ///
    /// It is not clear how this feature should be used. As of current testing, only sending `0x00`
    /// actually produces successful captures. Sending `0x01` does not work. More testing welcome.
    #[cfg(feature = "5mp")]
    pub fn set_auto_focus(&mut self, value: u8) -> Result<(), Error<SPI::Error, Delay::Error>> {
        self.write_reg(RegisterAddress::AutoFocus, value)?;
        self.wait_idle()
    }

    /// Sets the capture format of the camera.
    ///
    /// This function allows you to control what format the camera captures pictures in.
    /// `CaptureFormat::Jpeg` provides a good mix between image size and quality, and is the
    /// default.
    pub fn set_format(
        &mut self,
        format: CaptureFormat,
    ) -> Result<(), Error<SPI::Error, Delay::Error>> {
        self.write_reg(RegisterAddress::CaptureFormat, format as u8)?;
        self.wait_idle()
    }

    /// Sets the capture resolution of the camera.
    ///
    /// This function allows you to control the resolution of the pictures that are captured by the
    /// camera. Both the 3MP and 5MP cameras have two different default resolutions. See
    /// [`CaptureResolution`](CaptureResolution) for more details.
    pub fn set_resolution(
        &mut self,
        resolution: CaptureResolution,
    ) -> Result<(), Error<SPI::Error, Delay::Error>> {
        self.write_reg(
            RegisterAddress::CaptureResolution,
            resolution as u8 | WRITE_REGISTER_MASK,
        )?;
        self.wait_idle()
    }

    pub fn set_debug_device_address(
        &mut self,
        addr: u8,
    ) -> Result<(), Error<SPI::Error, Delay::Error>> {
        self.write_reg(RegisterAddress::DebugDeviceAddress, addr)?;
        self.wait_idle()
    }

    fn clear_fifo(&mut self) -> Result<(), Error<SPI::Error, Delay::Error>> {
        self.write_reg(RegisterAddress::ArduchipFifo, ArduchipCommand::Clear as u8)
    }

    fn capture_finished(&mut self) -> Result<bool, Error<SPI::Error, Delay::Error>> {
        let sensor_state = self.read_reg(RegisterAddress::SensorState)?;
        Ok((sensor_state & CAPTURE_FINISHED_MASK) != 0)
    }

    pub fn capture(&mut self) -> Result<(), Error<SPI::Error, Delay::Error>> {
        self.clear_fifo()?;
        self.write_reg(RegisterAddress::ArduchipFifo, ArduchipCommand::Start as u8)?;

        while !self.capture_finished()? {
            self.delay.delay_us(500u32).map_err(Error::Delay)?;
        }

        Ok(())
    }

    pub fn read_fifo_length(&mut self) -> Result<usize, Error<SPI::Error, Delay::Error>> {
        let size1 = self.read_reg(RegisterAddress::FifoSize1)? as usize;
        let size2 = self.read_reg(RegisterAddress::FifoSize2)? as usize;
        let size3 = self.read_reg(RegisterAddress::FifoSize3)? as usize;

        let length = (size3 << 16) | (size2 << 8) | size1;

        Ok(length)
    }

    pub fn read_fifo_byte(&mut self) -> Result<u8, Error<SPI::Error, Delay::Error>> {
        let output: [u8; 1] = [FIFO_READ_SINGLE];
        let mut data: [u8; 3] = [0; 3];
        self.spi
            .transfer(&mut data[..], &output[..])
            .map_err(Error::Spi)?;

        Ok(data[2])
    }

    pub fn read_fifo_full(
        &mut self,
        data: &mut [u8],
    ) -> Result<usize, Error<SPI::Error, Delay::Error>> {
        let length = data.len();

        self.spi
            .transaction(|bus| {
                let output: [u8; 1] = [FIFO_READ_BURST];

                let mut i = 0;
                while i <= (length - 63) {
                    // Send the read burst command and read 65 bytes
                    bus.transfer(&mut data[i..i + 65], &output[..])?;
                    // Ignore the first 2 bytes
                    data.copy_within(i + 2..i + 65, i);
                    i += 63;
                }

                bus.transfer(&mut data[i..], &output[..])?;
                data.copy_within(i + 2.., i);

                Ok(())
            })
            .map_err(Error::Spi)?;

        let marker = data
            .windows(2)
            .enumerate()
            .filter_map(|(i, p)| match p {
                [0xff, 0xd9] => Some(i),
                _ => None,
            })
            .last()
            .unwrap();

        Ok(marker + 2)
    }

    fn set_auto_camera_control(
        &mut self,
        cc: CameraControl,
        value: ControlValue,
    ) -> Result<(), Error<SPI::Error, Delay::Error>> {
        self.write_reg(
            RegisterAddress::GainExposureWhiteBalance,
            cc as u8 | value as u8,
        )?;
        self.wait_idle()
    }

    #[inline]
    pub fn enable_auto_white_balance(&mut self) -> Result<(), Error<SPI::Error, Delay::Error>> {
        self.set_auto_camera_control(CameraControl::WhiteBalance, ControlValue::Enable)
    }

    #[inline]
    pub fn disable_auto_white_balance(&mut self) -> Result<(), Error<SPI::Error, Delay::Error>> {
        self.set_auto_camera_control(CameraControl::WhiteBalance, ControlValue::Disable)
    }

    #[inline]
    pub fn enable_auto_iso(&mut self) -> Result<(), Error<SPI::Error, Delay::Error>> {
        self.set_auto_camera_control(CameraControl::Gain, ControlValue::Enable)
    }

    #[inline]
    pub fn disable_auto_iso(&mut self) -> Result<(), Error<SPI::Error, Delay::Error>> {
        self.set_auto_camera_control(CameraControl::Gain, ControlValue::Disable)
    }

    #[inline]
    pub fn enable_auto_exposure(&mut self) -> Result<(), Error<SPI::Error, Delay::Error>> {
        self.set_auto_camera_control(CameraControl::Exposure, ControlValue::Enable)
    }

    #[inline]
    pub fn disable_auto_exposure(&mut self) -> Result<(), Error<SPI::Error, Delay::Error>> {
        self.set_auto_camera_control(CameraControl::Exposure, ControlValue::Disable)
    }

    /// Fixes the white balance of the camera to `mode`
    ///
    /// This will set the white balance mode to the fixed value described by `mode`. The `auto`
    /// mode has not been tested yet, and is copied straight from the SDK provided by Arducam.
    /// This function ensures that auto white balance is disabled in the sensor by calling
    /// [`disable_auto_white_balance()`](ArducamMega::disable_auto_white_balance) first.
    pub fn set_white_balance_mode(
        &mut self,
        mode: WhiteBalanceMode,
    ) -> Result<(), Error<SPI::Error, Delay::Error>> {
        self.disable_auto_white_balance()?;
        self.write_reg(RegisterAddress::WhiteBalanceMode, mode as u8)?;
        self.wait_idle()
    }
}

#[derive(Copy, Clone, Debug)]
pub enum Error<SPI, Delay> {
    Spi(SPI),
    Delay(Delay),
}

#[cfg(test)]
mod tests {
    use super::*;
    use embedded_hal_mock::{delay, spi};

    #[test]
    fn new_works() {
        let _c = ArducamMega::new(spi::Mock::new(&[]), delay::MockNoop::new());
    }
}
