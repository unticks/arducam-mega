//! # `arducam-mega`
//!
//! `arducam-mega` is an [`embedded-hal`][hal] SPI driver for the [Arducam Mega][mega]. This driver
//! aims to provide access to all features of the camera. However, due to hardware access and time
//! constraints, not all features and hardware variants have been actively tested.
//!
//! [hal]: https://github.com/rust-embedded/embedded-hal
//! [mega]: https://www.arducam.com/camera-for-any-microcontroller/

#![cfg_attr(not(test), no_std)]

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
const CAPTURE_FINISHED_MASK: u8 = 0x04;
const FIFO_READ_BURST: u8 = 0x3c;
const FIFO_READ_SINGLE: u8 = 0x3d;

/// Represents the type of camera connected to the SPI bus
///
/// This enum is typically returned by [`ArducamMega::get_camera_type`], and indicates what kind of
/// camera was detected on the SPI bus.
///
/// **Please note**: The value `0x82` for `OV3640` is a guess based on the Arducam SDK. Please
/// submit an issue on GitHub if you have a 3MP Mega and can confirm that this works. Likewise, if
/// your camera reports the `Unknown(u8)` variant, please get in touch so we can add support for it
/// in the library.
#[derive(Debug, Clone, PartialEq)]
#[repr(u8)]
pub enum CameraType {
    /// Arducam Mega 5MP
    OV5640 = 0x81,
    /// Arducam Mega 3MP
    OV3640 = 0x82,
    Unknown(u8),
}

impl From<u8> for CameraType {
    fn from(id: u8) -> Self {
        match id {
            0x81 => CameraType::OV5640,
            0x82 => CameraType::OV3640,
            id => CameraType::Unknown(id),
        }
    }
}

#[derive(Debug, Clone)]
#[repr(u8)]
enum CameraControl {
    Gain = 0x00, // ISO
    Exposure = 0x01,
    WhiteBalance = 0x02,
}

#[derive(Debug, Clone)]
#[repr(u8)]
enum RegisterAddress {
    Power = 0x02,
    ArduchipFifo = 0x04,
    SensorReset = 0x07,
    DebugDeviceAddress = 0x0a,
    CaptureFormat = 0x20,
    CaptureResolution = 0x21,
    Brightness = 0x22,
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

/// Values to set the brightness bias of the camera
#[derive(Debug, Clone, Default)]
#[repr(u8)]
pub enum BrightnessLevel {
    #[default]
    Default = 0x00,
    PlusOne = 0x01,
    MinusOne = 0x02,
    PlusTwo = 0x03,
    MinusTwo = 0x04,
    PlusThree = 0x05,
    MinusThree = 0x06,
    PlusFour = 0x07,
    MinusFour = 0x08,
}

#[derive(Debug, Clone)]
#[repr(u8)]
enum PowerMode {
    LowPower = 0x07,
    Normal = 0x05,
}

#[derive(Debug, Clone)]
#[repr(u8)]
enum ControlValue {
    Disable = 0x00,
    Enable = 0x80,
}

/// The format in which the camera data should be formatted
///
/// You will most likely want to use `Jpeg`, as `Rgb` generates images that are much greater in
/// size than `Jpeg`, and could therefore data blobs that are too big for your MCU to handle.
#[derive(Debug, Clone, Default)]
#[repr(u8)]
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
#[repr(u8)]
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
#[repr(u8)]
enum ArduchipCommand {
    Clear = 0x01,
    Start = 0x02,
}

/// The white balance mode the camera should use
///
/// Whether you can use a static value or not will depend on the conditions in which your camera
/// operates. For most consistent results, it is recommended to use a specific mode, which will
/// prevent the camera from randomly switching from one mode to another.
#[derive(Debug, Clone, Default)]
#[repr(u8)]
pub enum WhiteBalanceMode {
    #[default]
    Auto = 0x00,
    Sunny = 0x01,
    Office = 0x02,
    Cloudy = 0x03,
    Home = 0x04,
}

/// The main ArducamMega struct
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

    /// Resets the camera sensor sensor
    ///
    /// This command sends a reset command to the camera. The Arducam SDK uses this after
    /// initialisation to ensure that the sensor is in a known-good state.
    pub fn reset(&mut self) -> Result<(), Error<SPI::Error, Delay::Error>> {
        self.write_reg(RegisterAddress::SensorReset, SENSOR_RESET_ENABLE)?;
        self.wait_idle()
    }

    pub fn get_camera_type(&mut self) -> Result<CameraType, Error<SPI::Error, Delay::Error>> {
        let id = self.read_reg(RegisterAddress::SensorId)?;

        Ok(id.into())
    }

    /// Sets the auto-focus of the camera
    ///
    /// It is not clear how this feature should be used. As of current testing, only sending `0x00`
    /// actually produces successful captures. Sending `0x01` makes the camera produce invalid
    /// image data. More testing welcome.
    #[cfg(feature = "5mp")]
    pub fn set_auto_focus(&mut self, value: u8) -> Result<(), Error<SPI::Error, Delay::Error>> {
        self.write_reg(RegisterAddress::AutoFocus, value)?;
        self.wait_idle()
    }

    /// Sets the capture format of the camera
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

    /// Sets the capture resolution of the camera
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

    /// Sets the camera's debug device address
    ///
    /// The Arducam SDK uses this command as part of the camera initialisation with the address
    /// `0x78`, however this does not appear to be necessary for the camera to function properly.
    pub fn set_debug_device_address(
        &mut self,
        addr: u8,
    ) -> Result<(), Error<SPI::Error, Delay::Error>> {
        self.write_reg(RegisterAddress::DebugDeviceAddress, addr)?;
        self.wait_idle()
    }

    /// Empties the camera's FIFO buffer
    ///
    /// This command empties the contents of the camera's FIFO buffer. This is used as part of the
    /// [`capture()`](ArducamMega::capture) function.
    fn clear_fifo(&mut self) -> Result<(), Error<SPI::Error, Delay::Error>> {
        self.write_reg(RegisterAddress::ArduchipFifo, ArduchipCommand::Clear as u8)
    }

    /// Checks whether the sensor has finished capturing an image
    ///
    /// This command reads a register in the camera to check if the sensor has finished taking a
    /// picture. You should not attempt to read the FIFO buffer length nor read any FIFO buffer
    /// data prior to this returning `true`.
    pub fn capture_finished(&mut self) -> Result<bool, Error<SPI::Error, Delay::Error>> {
        let sensor_state = self.read_reg(RegisterAddress::SensorState)?;
        Ok((sensor_state & CAPTURE_FINISHED_MASK) != 0)
    }

    /// Takes a picture using the currently-configured settings
    ///
    /// This command starts by emptying the camera's FIFO buffer and subsequently tells the sensor
    /// to capture an image. The image will be captured using the currently-configured settings
    /// (white balance, gain, exposure, colour filters, etc). This command blocks until the sensor
    /// has finished capturing the image.
    pub fn capture(&mut self) -> Result<(), Error<SPI::Error, Delay::Error>> {
        self.capture_noblock()?;

        while !self.capture_finished()? {
            self.delay.delay_us(500u32).map_err(Error::Delay)?;
        }

        Ok(())
    }

    /// Non-blocking version of [`capture()`](ArducamMega::capture)
    pub fn capture_noblock(&mut self) -> Result<(), Error<SPI::Error, Delay::Error>> {
        self.clear_fifo()?;
        self.write_reg(RegisterAddress::ArduchipFifo, ArduchipCommand::Start as u8)?;

        Ok(())
    }

    /// Reads the size of the camera's FIFO buffer length
    ///
    /// This function reads out the size of the FIFO buffer length. This (roughly) represents the
    /// size of the picture. It appears that in some cases, the reported buffer is larger than the
    /// actual picture data.
    pub fn read_fifo_length(&mut self) -> Result<usize, Error<SPI::Error, Delay::Error>> {
        let size1 = self.read_reg(RegisterAddress::FifoSize1)? as usize;
        let size2 = self.read_reg(RegisterAddress::FifoSize2)? as usize;
        let size3 = self.read_reg(RegisterAddress::FifoSize3)? as usize;

        let length = (size3 << 16) | (size2 << 8) | size1;

        Ok(length)
    }

    /// Reads a single byte out of the FIFO buffer
    ///
    /// Returns the first byte out of the FIFO buffer. After it has been read, the FIFO buffer will
    /// advance and the byte will be gone from the camera. In theory, calling this function
    /// [`fifo_length`](ArducamMega::read_fifo_length) times should provide you with the full
    /// picture data.
    ///
    /// Reading out the entire image data like this will be relatively slow, as each byte transfer
    /// will require an SPI transaction to be setup and ended. For faster transfers, please see
    /// [`read_fifo_full`](ArducamMega::read_fifo_full).
    pub fn read_fifo_byte(&mut self) -> Result<u8, Error<SPI::Error, Delay::Error>> {
        let output: [u8; 1] = [FIFO_READ_SINGLE];
        let mut data: [u8; 3] = [0; 3];
        self.spi
            .transfer(&mut data[..], &output[..])
            .map_err(Error::Spi)?;

        Ok(data[2])
    }

    /// Reads out the entire FIFO buffer
    ///
    /// Reads out the camera's entire FIFO buffer into `data`. `data` must be large enough to
    /// accomodate the entire data transfer, or some data loss will occur. This function currently
    /// attemps to find the end-of-file JPEG marker in the byte-stream and returns its location.
    /// This is not currently tested for other data formats (YUV, RGB).
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
                [0xff, 0xd9] => Some(i + 2),
                _ => None,
            })
            .last()
            .unwrap_or(length);

        Ok(marker)
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

    /// Enables the camera's auto white balance
    ///
    /// **Note**: This appears to not work on the ArducamMega 5MP, where pictures are severely
    /// green-tinted when using AWB.
    #[inline]
    pub fn enable_auto_white_balance(&mut self) -> Result<(), Error<SPI::Error, Delay::Error>> {
        self.set_auto_camera_control(CameraControl::WhiteBalance, ControlValue::Enable)
    }

    /// Disables the camera's auto white balance
    ///
    /// This function is automatically called by
    /// [`set_white_balance_mode()`](ArducamMega::set_white_balance_mode).
    #[inline]
    pub fn disable_auto_white_balance(&mut self) -> Result<(), Error<SPI::Error, Delay::Error>> {
        self.set_auto_camera_control(CameraControl::WhiteBalance, ControlValue::Disable)
    }

    /// Enables the camera's automatic gain adjustment
    #[inline]
    pub fn enable_auto_iso(&mut self) -> Result<(), Error<SPI::Error, Delay::Error>> {
        self.set_auto_camera_control(CameraControl::Gain, ControlValue::Enable)
    }

    /// Disables the camera's automatic gain adjustment
    #[inline]
    pub fn disable_auto_iso(&mut self) -> Result<(), Error<SPI::Error, Delay::Error>> {
        self.set_auto_camera_control(CameraControl::Gain, ControlValue::Disable)
    }

    /// Enables the camera's automatic exposure control
    #[inline]
    pub fn enable_auto_exposure(&mut self) -> Result<(), Error<SPI::Error, Delay::Error>> {
        self.set_auto_camera_control(CameraControl::Exposure, ControlValue::Enable)
    }

    /// Disables the camera's automatic exposure control
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

    #[inline]
    fn set_power_mode(&mut self, mode: PowerMode) -> Result<(), Error<SPI::Error, Delay::Error>> {
        self.write_reg(RegisterAddress::Power, mode as u8)
    }

    /// Turns on the camera's low power mode
    #[inline]
    pub fn enable_low_power_mode(&mut self) -> Result<(), Error<SPI::Error, Delay::Error>> {
        self.set_power_mode(PowerMode::LowPower)
    }

    /// Turns off the camera's low power mode
    #[inline]
    pub fn disable_low_power_mode(&mut self) -> Result<(), Error<SPI::Error, Delay::Error>> {
        self.set_power_mode(PowerMode::Normal)
    }

    /// Sets the camera's brightness bias
    pub fn set_brightness_bias(
        &mut self,
        level: BrightnessLevel,
    ) -> Result<(), Error<SPI::Error, Delay::Error>> {
        self.write_reg(RegisterAddress::Brightness, level as u8)?;
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
    use embedded_hal_mock::{
        delay,
        spi::{self, Transaction},
    };

    #[test]
    fn read_reg_returns_third_byte() {
        let expectations = [
            Transaction::transaction_start(),
            Transaction::transfer(vec![0x45], vec![0x00, 0x00, 0x01]),
            Transaction::transaction_end(),
        ];
        let mut spi = spi::Mock::new(&expectations);
        let mut c = ArducamMega::new(&mut spi, delay::MockNoop::new());
        assert_eq!(c.read_reg(RegisterAddress::FifoSize1).unwrap(), 0x01);
        spi.done();
    }

    #[test]
    fn write_reg_transforms_addr() {
        let expectations = [
            Transaction::transaction_start(),
            Transaction::write_vec(vec![0xc5, 0x02]),
            Transaction::transaction_end(),
        ];
        let mut spi = spi::Mock::new(&expectations);
        let mut c = ArducamMega::new(&mut spi, delay::MockNoop::new());
        c.write_reg(RegisterAddress::FifoSize1, 0x02).unwrap();
        spi.done();
    }

    #[test]
    fn reset_sends_correct_data() {
        let expectations = [
            Transaction::transaction_start(),
            Transaction::write_vec(vec![0x87, 0x40]),
            Transaction::transaction_end(),
            // wait_idle
            Transaction::transaction_start(),
            Transaction::transfer(vec![0x44], vec![0x00, 0x00, 0x02]),
            Transaction::transaction_end(),
        ];
        let mut spi = spi::Mock::new(&expectations);
        let mut c = ArducamMega::new(&mut spi, delay::MockNoop::new());
        c.reset().unwrap();
        spi.done();
    }

    #[test]
    fn wait_idle_returns_immediately_if_idle() {
        let expectations = [
            Transaction::transaction_start(),
            Transaction::transfer(vec![0x44], vec![0x00, 0x00, 0x02]),
            Transaction::transaction_end(),
        ];
        let mut spi = spi::Mock::new(&expectations);
        let mut c = ArducamMega::new(&mut spi, delay::MockNoop::new());
        c.wait_idle().unwrap();
        spi.done();
    }

    #[test]
    fn wait_idle_waits_until_idle() {
        let expectations = [
            Transaction::transaction_start(),
            Transaction::transfer(vec![0x44], vec![0x00, 0x00, 0x01]),
            Transaction::transaction_end(),
            Transaction::transaction_start(),
            Transaction::transfer(vec![0x44], vec![0x00, 0x00, 0x01]),
            Transaction::transaction_end(),
            Transaction::transaction_start(),
            Transaction::transfer(vec![0x44], vec![0x00, 0x00, 0x01]),
            Transaction::transaction_end(),
            Transaction::transaction_start(),
            Transaction::transfer(vec![0x44], vec![0x01, 0x01, 0x02]),
            Transaction::transaction_end(),
        ];
        let mut spi = spi::Mock::new(&expectations);
        let mut c = ArducamMega::new(&mut spi, delay::MockNoop::new());
        c.wait_idle().unwrap();
        spi.done();
    }

    #[test]
    fn get_camera_type_reads_a_register_and_returns_an_enum() {
        let expectations = [
            Transaction::transaction_start(),
            Transaction::transfer(vec![0x40], vec![0x00, 0x00, 0x81]),
            Transaction::transaction_end(),
            Transaction::transaction_start(),
            Transaction::transfer(vec![0x40], vec![0x00, 0x00, 0x82]),
            Transaction::transaction_end(),
            Transaction::transaction_start(),
            Transaction::transfer(vec![0x40], vec![0x00, 0x00, 0x33]),
            Transaction::transaction_end(),
        ];
        let mut spi = spi::Mock::new(&expectations);
        let mut c = ArducamMega::new(&mut spi, delay::MockNoop::new());
        assert_eq!(c.get_camera_type().unwrap(), CameraType::OV5640);
        assert_eq!(c.get_camera_type().unwrap(), CameraType::OV3640);
        assert_eq!(c.get_camera_type().unwrap(), CameraType::Unknown(0x33));
        spi.done();
    }

    #[cfg(feature = "5mp")]
    #[test]
    fn set_auto_focus_writes_a_register() {
        let expectations = [
            Transaction::transaction_start(),
            Transaction::write_vec(vec![0xa9, 0x33]),
            Transaction::transaction_end(),
            // wait_idle
            Transaction::transaction_start(),
            Transaction::transfer(vec![0x44], vec![0x00, 0x00, 0x02]),
            Transaction::transaction_end(),
        ];
        let mut spi = spi::Mock::new(&expectations);
        let mut c = ArducamMega::new(&mut spi, delay::MockNoop::new());
        c.set_auto_focus(0x33).unwrap();
        spi.done();
    }

    #[test]
    fn set_format_writes_a_register() {
        let expectations = [
            Transaction::transaction_start(),
            Transaction::write_vec(vec![0xa0, 0x01]),
            Transaction::transaction_end(),
            // wait_idle
            Transaction::transaction_start(),
            Transaction::transfer(vec![0x44], vec![0x00, 0x00, 0x02]),
            Transaction::transaction_end(),
        ];
        let mut spi = spi::Mock::new(&expectations);
        let mut c = ArducamMega::new(&mut spi, delay::MockNoop::new());
        c.set_format(CaptureFormat::default()).unwrap();
        spi.done();
    }

    #[test]
    fn set_resolution_writes_a_register() {
        let expectations = [
            Transaction::transaction_start(),
            Transaction::write_vec(vec![0xa1, 0x84]),
            Transaction::transaction_end(),
            // wait_idle
            Transaction::transaction_start(),
            Transaction::transfer(vec![0x44], vec![0x00, 0x00, 0x02]),
            Transaction::transaction_end(),
        ];
        let mut spi = spi::Mock::new(&expectations);
        let mut c = ArducamMega::new(&mut spi, delay::MockNoop::new());
        c.set_resolution(CaptureResolution::Hd).unwrap();
        spi.done();
    }

    #[test]
    fn set_debug_device_address_writes_a_register() {
        let expectations = [
            Transaction::transaction_start(),
            Transaction::write_vec(vec![0x8a, 0x33]),
            Transaction::transaction_end(),
            // wait_idle
            Transaction::transaction_start(),
            Transaction::transfer(vec![0x44], vec![0x00, 0x00, 0x02]),
            Transaction::transaction_end(),
        ];
        let mut spi = spi::Mock::new(&expectations);
        let mut c = ArducamMega::new(&mut spi, delay::MockNoop::new());
        c.set_debug_device_address(0x33).unwrap();
        spi.done();
    }

    #[test]
    fn clear_fifo_writes_a_register() {
        let expectations = [
            Transaction::transaction_start(),
            Transaction::write_vec(vec![0x84, 0x01]),
            Transaction::transaction_end(),
        ];
        let mut spi = spi::Mock::new(&expectations);
        let mut c = ArducamMega::new(&mut spi, delay::MockNoop::new());
        c.clear_fifo().unwrap();
        spi.done();
    }

    #[test]
    fn capture_finished_detects_unfinished() {
        let expectations = [
            Transaction::transaction_start(),
            Transaction::transfer(vec![0x44], vec![0x00, 0x00, 0xfb]),
            Transaction::transaction_end(),
        ];
        let mut spi = spi::Mock::new(&expectations);
        let mut c = ArducamMega::new(&mut spi, delay::MockNoop::new());
        assert!(!c.capture_finished().unwrap());
        spi.done();
    }

    #[test]
    fn capture_finished_detects_finished() {
        let expectations = [
            Transaction::transaction_start(),
            Transaction::transfer(vec![0x44], vec![0x00, 0x00, 0x04]),
            Transaction::transaction_end(),
        ];
        let mut spi = spi::Mock::new(&expectations);
        let mut c = ArducamMega::new(&mut spi, delay::MockNoop::new());
        assert!(c.capture_finished().unwrap());
        spi.done();
    }

    #[test]
    fn capture_no_block_clears_fifo_and_writes_reg() {
        let expectations = [
            Transaction::transaction_start(),
            Transaction::write_vec(vec![0x84, 0x01]),
            Transaction::transaction_end(),
            Transaction::transaction_start(),
            Transaction::write_vec(vec![0x84, 0x02]),
            Transaction::transaction_end(),
        ];
        let mut spi = spi::Mock::new(&expectations);
        let mut c = ArducamMega::new(&mut spi, delay::MockNoop::new());
        c.capture_noblock().unwrap();
        spi.done();
    }

    #[test]
    fn capture_returns_instantly_after_capture_finishes() {
        let expectations = [
            Transaction::transaction_start(),
            Transaction::write_vec(vec![0x84, 0x01]),
            Transaction::transaction_end(),
            Transaction::transaction_start(),
            Transaction::write_vec(vec![0x84, 0x02]),
            Transaction::transaction_end(),
            Transaction::transaction_start(),
            Transaction::transfer(vec![0x44], vec![0x00, 0x00, 0x04]),
            Transaction::transaction_end(),
        ];
        let mut spi = spi::Mock::new(&expectations);
        let mut c = ArducamMega::new(&mut spi, delay::MockNoop::new());
        c.capture().unwrap();
        spi.done();
    }

    #[test]
    fn capture_blocks_until_capture_finished() {
        let expectations = [
            Transaction::transaction_start(),
            Transaction::write_vec(vec![0x84, 0x01]),
            Transaction::transaction_end(),
            Transaction::transaction_start(),
            Transaction::write_vec(vec![0x84, 0x02]),
            Transaction::transaction_end(),
            Transaction::transaction_start(),
            Transaction::transfer(vec![0x44], vec![0x00, 0x00, 0x00]),
            Transaction::transaction_end(),
            Transaction::transaction_start(),
            Transaction::transfer(vec![0x44], vec![0x00, 0x00, 0x00]),
            Transaction::transaction_end(),
            Transaction::transaction_start(),
            Transaction::transfer(vec![0x44], vec![0x00, 0x00, 0x00]),
            Transaction::transaction_end(),
            Transaction::transaction_start(),
            Transaction::transfer(vec![0x44], vec![0x00, 0x00, 0x04]),
            Transaction::transaction_end(),
        ];
        let mut spi = spi::Mock::new(&expectations);
        let mut c = ArducamMega::new(&mut spi, delay::MockNoop::new());
        c.capture().unwrap();
        spi.done();
    }

    #[test]
    fn read_fifo_length_reads_three_regs() {
        let expectations = [
            Transaction::transaction_start(),
            Transaction::transfer(vec![0x45], vec![0x00, 0x00, 0xab]),
            Transaction::transaction_end(),
            Transaction::transaction_start(),
            Transaction::transfer(vec![0x46], vec![0x00, 0x00, 0xbc]),
            Transaction::transaction_end(),
            Transaction::transaction_start(),
            Transaction::transfer(vec![0x47], vec![0x00, 0x00, 0xcd]),
            Transaction::transaction_end(),
        ];
        let mut spi = spi::Mock::new(&expectations);
        let mut c = ArducamMega::new(&mut spi, delay::MockNoop::new());
        assert_eq!(c.read_fifo_length().unwrap(), 13483179);
        spi.done();
    }

    #[test]
    fn read_fifo_byte_reads_a_register() {
        let expectations = [
            Transaction::transaction_start(),
            Transaction::transfer(vec![0x3d], vec![0x00, 0x00, 0x33]),
            Transaction::transaction_end(),
        ];
        let mut spi = spi::Mock::new(&expectations);
        let mut c = ArducamMega::new(&mut spi, delay::MockNoop::new());
        assert_eq!(c.read_fifo_byte().unwrap(), 0x33);
        spi.done();
    }

    #[test]
    fn read_fifo_full_skips_first_two_bytes() {
        let mut buffer = [0; 65];
        let expectations = [
            Transaction::transaction_start(),
            Transaction::transfer(
                vec![0x3c],
                vec![
                    0x00, 0x00, 0x33, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
                    0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
                    0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
                    0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
                    0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
                ],
            ),
            Transaction::transfer(vec![0x3c], vec![0x00, 0x00]),
            Transaction::transaction_end(),
        ];
        let mut spi = spi::Mock::new(&expectations);
        let mut c = ArducamMega::new(&mut spi, delay::MockNoop::new());
        c.read_fifo_full(&mut buffer).unwrap();
        assert_eq!(buffer[0], 0x33);
        assert_eq!(buffer.iter().map(|i| *i as u32).sum::<u32>(), 113);
        spi.done();
    }

    #[test]
    fn read_fifo_full_detects_eof_jpeg_marker() {
        let mut buffer = [0; 65];
        let expectations = [
            Transaction::transaction_start(),
            Transaction::transfer(
                vec![0x3c],
                vec![
                    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xd9, 0x00, 0x00, 0x00, 0x00,
                    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                ],
            ),
            Transaction::transfer(vec![0x3c], vec![0x00, 0x00]),
            Transaction::transaction_end(),
        ];
        let mut spi = spi::Mock::new(&expectations);
        let mut c = ArducamMega::new(&mut spi, delay::MockNoop::new());
        let pos = c.read_fifo_full(&mut buffer).unwrap();
        assert_eq!(buffer[pos - 2], 0xff);
        assert_eq!(buffer[pos - 1], 0xd9);
        spi.done();
    }

    #[test]
    fn set_auto_camera_control_writes_a_register() {
        let expectations = [
            Transaction::transaction_start(),
            Transaction::write_vec(vec![0xaa, 0x81]),
            Transaction::transaction_end(),
            // wait_idle
            Transaction::transaction_start(),
            Transaction::transfer(vec![0x44], vec![0x00, 0x00, 0x02]),
            Transaction::transaction_end(),
        ];
        let mut spi = spi::Mock::new(&expectations);
        let mut c = ArducamMega::new(&mut spi, delay::MockNoop::new());
        c.set_auto_camera_control(CameraControl::Exposure, ControlValue::Enable)
            .unwrap();
        spi.done();
    }

    #[test]
    fn enable_auto_white_balance_writes_a_register() {
        let expectations = [
            Transaction::transaction_start(),
            Transaction::write_vec(vec![0xaa, 0x82]),
            Transaction::transaction_end(),
            // wait_idle
            Transaction::transaction_start(),
            Transaction::transfer(vec![0x44], vec![0x00, 0x00, 0x02]),
            Transaction::transaction_end(),
        ];
        let mut spi = spi::Mock::new(&expectations);
        let mut c = ArducamMega::new(&mut spi, delay::MockNoop::new());
        c.enable_auto_white_balance().unwrap();
        spi.done();
    }

    #[test]
    fn disable_auto_white_balance_writes_a_register() {
        let expectations = [
            Transaction::transaction_start(),
            Transaction::write_vec(vec![0xaa, 0x02]),
            Transaction::transaction_end(),
            // wait_idle
            Transaction::transaction_start(),
            Transaction::transfer(vec![0x44], vec![0x00, 0x00, 0x02]),
            Transaction::transaction_end(),
        ];
        let mut spi = spi::Mock::new(&expectations);
        let mut c = ArducamMega::new(&mut spi, delay::MockNoop::new());
        c.disable_auto_white_balance().unwrap();
        spi.done();
    }

    #[test]
    fn enable_auto_iso_writes_a_register() {
        let expectations = [
            Transaction::transaction_start(),
            Transaction::write_vec(vec![0xaa, 0x80]),
            Transaction::transaction_end(),
            // wait_idle
            Transaction::transaction_start(),
            Transaction::transfer(vec![0x44], vec![0x00, 0x00, 0x02]),
            Transaction::transaction_end(),
        ];
        let mut spi = spi::Mock::new(&expectations);
        let mut c = ArducamMega::new(&mut spi, delay::MockNoop::new());
        c.enable_auto_iso().unwrap();
        spi.done();
    }

    #[test]
    fn disable_auto_iso_writes_a_register() {
        let expectations = [
            Transaction::transaction_start(),
            Transaction::write_vec(vec![0xaa, 0x00]),
            Transaction::transaction_end(),
            // wait_idle
            Transaction::transaction_start(),
            Transaction::transfer(vec![0x44], vec![0x00, 0x00, 0x02]),
            Transaction::transaction_end(),
        ];
        let mut spi = spi::Mock::new(&expectations);
        let mut c = ArducamMega::new(&mut spi, delay::MockNoop::new());
        c.disable_auto_iso().unwrap();
        spi.done();
    }

    #[test]
    fn enable_auto_exposure_writes_a_register() {
        let expectations = [
            Transaction::transaction_start(),
            Transaction::write_vec(vec![0xaa, 0x81]),
            Transaction::transaction_end(),
            // wait_idle
            Transaction::transaction_start(),
            Transaction::transfer(vec![0x44], vec![0x00, 0x00, 0x02]),
            Transaction::transaction_end(),
        ];
        let mut spi = spi::Mock::new(&expectations);
        let mut c = ArducamMega::new(&mut spi, delay::MockNoop::new());
        c.enable_auto_exposure().unwrap();
        spi.done();
    }

    #[test]
    fn disable_auto_exposure_writes_a_register() {
        let expectations = [
            Transaction::transaction_start(),
            Transaction::write_vec(vec![0xaa, 0x01]),
            Transaction::transaction_end(),
            // wait_idle
            Transaction::transaction_start(),
            Transaction::transfer(vec![0x44], vec![0x00, 0x00, 0x02]),
            Transaction::transaction_end(),
        ];
        let mut spi = spi::Mock::new(&expectations);
        let mut c = ArducamMega::new(&mut spi, delay::MockNoop::new());
        c.disable_auto_exposure().unwrap();
        spi.done();
    }

    #[test]
    fn set_white_balance_mode_disables_autowb_and_writes_a_reg() {
        let expectations = [
            Transaction::transaction_start(),
            Transaction::write_vec(vec![0xaa, 0x02]),
            Transaction::transaction_end(),
            // wait_idle
            Transaction::transaction_start(),
            Transaction::transfer(vec![0x44], vec![0x00, 0x00, 0x02]),
            Transaction::transaction_end(),
            Transaction::transaction_start(),
            Transaction::write_vec(vec![0xa6, 0x04]),
            Transaction::transaction_end(),
            // wait_idle
            Transaction::transaction_start(),
            Transaction::transfer(vec![0x44], vec![0x00, 0x00, 0x02]),
            Transaction::transaction_end(),
        ];
        let mut spi = spi::Mock::new(&expectations);
        let mut c = ArducamMega::new(&mut spi, delay::MockNoop::new());
        c.set_white_balance_mode(WhiteBalanceMode::Home).unwrap();
        spi.done();
    }

    #[test]
    fn enable_low_power_mode_writes_a_reg() {
        let expectations = [
            Transaction::transaction_start(),
            Transaction::write_vec(vec![0x82, 0x07]),
            Transaction::transaction_end(),
        ];
        let mut spi = spi::Mock::new(&expectations);
        let mut c = ArducamMega::new(&mut spi, delay::MockNoop::new());
        c.enable_low_power_mode().unwrap();
        spi.done();
    }

    #[test]
    fn disable_low_power_mode_writes_a_reg() {
        let expectations = [
            Transaction::transaction_start(),
            Transaction::write_vec(vec![0x82, 0x05]),
            Transaction::transaction_end(),
        ];
        let mut spi = spi::Mock::new(&expectations);
        let mut c = ArducamMega::new(&mut spi, delay::MockNoop::new());
        c.disable_low_power_mode().unwrap();
        spi.done();
    }

    #[test]
    fn set_brightness_bias_writes_a_reg() {
        let expectations = [
            Transaction::transaction_start(),
            Transaction::write_vec(vec![0xa2, 0x03]),
            Transaction::transaction_end(),
            // wait_idle
            Transaction::transaction_start(),
            Transaction::transfer(vec![0x44], vec![0x00, 0x00, 0x02]),
            Transaction::transaction_end(),
        ];
        let mut spi = spi::Mock::new(&expectations);
        let mut c = ArducamMega::new(&mut spi, delay::MockNoop::new());
        c.set_brightness_bias(BrightnessLevel::PlusTwo).unwrap();
        spi.done();
    }
}
