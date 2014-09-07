

const ERR_NO_DEVICE = "The device at I2C address 0x%02x is disabled.";
const ERR_I2C_READ = "I2C Read Failure. Device: 0x%02x Register: 0x%02x";
const ERR_BAD_TIMER = "You have to start %s with an interval and callback";
const ERR_WRONG_DEVICE = "The device at I2C address 0x%02x is not a %s.";


class SX1509 {

    //Private variables
    _i2c       = null;
    _addr      = null;
    _callbacks = null;
    _int_pin   = null;
    
    // I/O Expander internal registers
    static BANK_A = {   REGDATA    = 0x11,
                        REGDIR     = 0x0F,
                        REGPULLUP  = 0x07,
                        REGPULLDN  = 0x09,
                        REGINTMASK = 0x13,
                        REGSNSHI   = 0x16,
                        REGSNSLO   = 0x17,
                        REGINTSRC  = 0x19,
                        REGINPDIS  = 0x01,
                        REGOPENDRN = 0x0B,
                        REGLEDDRV  = 0x21,
                        REGCLOCK   = 0x1E,
                        REGMISC    = 0x1F,
                        REGRESET   = 0x7D}

    static BANK_B = {   REGDATA    = 0x10,
                        REGDIR     = 0x0E,
                        REGPULLUP  = 0x06,
                        REGPULLDN  = 0x08,
                        REGINTMASK = 0x12,
                        REGSNSHI   = 0x14,
                        REGSNSLO   = 0x15,
                        REGINTSRC  = 0x18,
                        REGINPDIS  = 0x00,
                        REGOPENDRN = 0x0A,
                        REGLEDDRV  = 0x20,
                        REGCLOCK   = 0x1E,
                        REGMISC    = 0x1F,
                        REGRESET   = 0x7D}

    // Constructor requires the i2c bus, the address on that bus and the hardware pin to use for interrupts
    // These should all be configured before use here.
    constructor(i2c, address, int_pin){
        _i2c  = i2c;
        _addr = address;
        _callbacks = [];
        _callbacks.resize(16, null);
        _int_pin = int_pin;

        reset();
        clearAllIrqs();
    }
    

    // ---- Low level functions ----

    // Reads a single byte from a registry
    function readReg(register) {
        local data = _i2c.read(_addr, format("%c", register), 1);
        if (data == null) {
            server.error(format(ERR_I2C_READ, _addr, register));
            return -1;
        }
        return data[0];
    }
    
    // Writes a single byte to a registry
    function writeReg(register, data) {
        _i2c.write(_addr, format("%c%c", register, data));
        // server.log(format("Setting device 0x%02X register 0x%02X to 0x%02X", _addr, register, data));
    }
    
    // Changes one bit out of the selected register (byte)
    function writeBit(register, bitn, level) {
        local value = readReg(register);
        value = (level == 0)?(value & ~(1<<bitn)):(value | (1<<bitn));
        writeReg(register, value);
    }
    
    // Writes a registry but masks specific bits. Similar to writeBit but for multiple bits.
    function writeMasked(register, data, mask) {
        local value = readReg(register);
        value = (value & ~mask) | (data & mask);
        writeReg(register, value);
    }

    // set or clear a selected GPIO pin, 0-16
    function setPin(gpio, level) {
        writeBit(bank(gpio).REGDATA, gpio % 8, level ? 1 : 0);
    }

    // configure specified GPIO pin as input(0) or output(1)
    function setDir(gpio, output) {
        writeBit(bank(gpio).REGDIR, gpio % 8, output ? 0 : 1);
    }

    // enable or disable input buffers
    function setInputBuffer(gpio, enable) {
        writeBit(bank(gpio).REGINPDIS, gpio % 8, enable ? 0 : 1);
    }

    // enable or disable open drain
    function setOpenDrain(gpio, enable) {
        writeBit(bank(gpio).REGOPENDRN, gpio % 8, enable ? 1 : 0);
    }
    
    // enable or disable internal pull up resistor for specified GPIO
    function setPullUp(gpio, enable) {
        writeBit(bank(gpio).REGPULLUP, gpio % 8, enable ? 1 : 0);
    }
    
    // enable or disable internal pull down resistor for specified GPIO
    function setPullDn(gpio, enable) {
        writeBit(bank(gpio).REGPULLDN, gpio % 8, enable ? 1 : 0);
    }

    // configure whether specified GPIO will trigger an interrupt
    function setIrqMask(gpio, enable) {
        writeBit(bank(gpio).REGINTMASK, gpio % 8, enable ? 0 : 1);
    }

    // clear interrupt on specified GPIO
    function clearIrq(gpio) {
        writeBit(bank(gpio).REGINTMASK, gpio % 8, 1);
    }

    // get state of specified GPIO
    function getPin(gpio) {
        return ((readReg(bank(gpio).REGDATA) & (1<<(gpio%8))) ? 1 : 0);
    }

    // resets the device with a software reset
    function reboot() {
        writeReg(bank(0).REGRESET, 0x12);
        writeReg(bank(0).REGRESET, 0x34);
    }

    // configure which callback should be called for each pin transition
    function setCallback(gpio, _callback) {
        _callbacks[gpio] = _callback;
        
        // Initialize the interrupt Pin
        hardware.pin1.configure(DIGITAL_IN_PULLUP, fire_callback.bindenv(this));
    }

    // finds and executes the callback after the irq pin (pin 1) fires
    function fire_callback() {
        local irq = getIrq();
        clearAllIrqs();
        for (local i = 0; i < 16; i++){
            if ( (irq & (1 << i)) && (typeof _callbacks[i] == "function")){
                _callbacks[i](getPin(i)); 
            }
        }
    }

    
    // ---- High level functions ----


    // Write registers to default values
    function reset(){
        writeReg(BANK_A.REGDIR, 0xFF);
        writeReg(BANK_A.REGDATA, 0xFF);
        writeReg(BANK_A.REGPULLUP, 0x00);
        writeReg(BANK_A.REGPULLDN, 0x00);
        writeReg(BANK_A.REGINTMASK, 0xFF);
        writeReg(BANK_A.REGSNSHI, 0x00);
        writeReg(BANK_A.REGSNSLO, 0x00);
        
        writeReg(BANK_B.REGDIR, 0xFF);
        writeReg(BANK_B.REGDATA, 0xFF);
        writeReg(BANK_B.REGPULLUP, 0x00);
        writeReg(BANK_B.REGPULLDN, 0x00);
        writeReg(BANK_A.REGINTMASK, 0xFF);
        writeReg(BANK_B.REGSNSHI, 0x00);
        writeReg(BANK_B.REGSNSLO, 0x00);
    }

    // Returns the register numbers for the bank that the given gpio is on
    function bank(gpio){
        return (gpio > 7) ? BANK_B : BANK_A;
    }

    // configure whether edges trigger an interrupt for specified GPIO
    function setIrqEdges( gpio, rising, falling) {
        local bank = bank(gpio);
        gpio = gpio % 8;
        local mask = 0x03 << ((gpio & 3) << 1);
        local data = (2*falling + rising) << ((gpio & 3) << 1);
        writeMasked(gpio >= 4 ? bank.REGSNSHI : bank.REGSNSLO, data, mask);
    }

    // Resets all the IRQs
    function clearAllIrqs() {
        writeReg(BANK_A.REGINTSRC,0xff);
        writeReg(BANK_B.REGINTSRC,0xff);
    }

    // Read all the IRQs as a single 16-bit bitmap
    function getIrq(){
        return ((readReg(BANK_B.REGINTSRC) & 0xFF) << 8) | (readReg(BANK_A.REGINTSRC) & 0xFF);
    }
    
    // sets the clock 
    function setClock(gpio, enable) {
        writeReg(bank(gpio).REGCLOCK, enable ? 0x50 : 0x00); // 2mhz internal oscillator 
    }
    
    // enable or disable the LED drivers
    function setLEDDriver(gpio, enable) {
        writeBit(bank(gpio).REGLEDDRV, gpio & 7, enable ? 1 : 0);
        writeReg(bank(gpio).REGMISC, 0x70); // Set clock to 2mhz / (2 ^ (1-1)) = 2mhz, use linear fading
    }
    
    // sets the Time On value for the LED register
    function setTimeOn(gpio, value) {
        writeReg(gpio<4 ? 0x29+gpio*3 : 0x35+(gpio-4)*5, value)
    }
    
    // sets the On Intensity level LED register
    function setIntensityOn(gpio, value) {
        writeReg(gpio<4 ? 0x2A+gpio*3 : 0x36+(gpio-4)*5, value)
    }
    
    // sets the Time Off value for the LED register
    function setOff(gpio, value) {
        writeReg(gpio<4 ? 0x2B+gpio*3 : 0x37+(gpio-4)*5, value)
    }
    
    // sets the Rise Time value for the LED register
    function setRiseTime(gpio, value) {
        if (gpio % 8 < 4) return; // Can't do all pins
        writeReg(gpio<12 ? 0x38+(gpio-4)*5 : 0x58+(gpio-12)*5, value)
    }
    
    // sets the Fall Time value for the LED register
    function setFallTime(gpio, value) {
        if (gpio % 8 < 4) return; // Can't do all pins
        writeReg(gpio<12 ? 0x39+(gpio-4)*5 : 0x59+(gpio-12)*5, value)
    }   
}

class ExpGPIO {
    _expander = null;  //Instance of an Expander class
    _gpio     = null;  //Pin number of this GPIO pin
    _mode     = null;  //The mode configured for this pin
    
    // This definition augments the pin configuration constants as defined in:
    // http://electricimp.com/docs/api/hardware/pin/configure/
    static LED_OUT = 1000001;
    
    // Constructor requires the IO Expander class and the pin number to aquire
    constructor(expander, gpio) {
        _expander = expander;
        _gpio     = gpio;
    }
    
    //Optional initial state (defaults to 0 just like the imp)
    function configure(mode, param = null) {
        _mode = mode;
        
        if (mode == DIGITAL_OUT) {
            // Digital out - Param is the initial value of the pin
            // Set the direction, turn off the pull up and enable the pin
            _expander.setDir(_gpio,1);
            _expander.setPullUp(_gpio,0);
            if(param != null) {
                _expander.setPin(_gpio, param);    
            } else {
                _expander.setPin(_gpio, 0);
            }
            
            return this;
        } else if (mode == ExpGPIO.LED_OUT) {
            // LED out - Param is the initial intensity
            // Set the direction, turn off the pull up and enable the pin
            // Configure a bunch of other LED specific timers and settings
            _expander.setPullUp(_gpio, 0);
            _expander.setInputBuffer(_gpio, 0);
            _expander.setOpenDrain(_gpio, 1);
            _expander.setDir(_gpio, 1);
            _expander.setClock(_gpio, 1);
            _expander.setLEDDriver(_gpio, 1);
            _expander.setTimeOn(_gpio, 0);
            _expander.setOff(_gpio, 0);
            _expander.setRiseTime(_gpio, 0);
            _expander.setFallTime(_gpio, 0);
            _expander.setIntensityOn(_gpio, param > 0 ? param : 0);
            _expander.setPin(_gpio, param > 0 ? 0 : 1);
            
            return this;
        } else if (mode == DIGITAL_IN) {
            // Digital in - Param is the callback function
            // Set the direction and disable to pullup
            _expander.setDir(_gpio,0);
            _expander.setPullUp(_gpio,0);
            // Fall through to the callback setup
        } else if (mode == DIGITAL_IN_PULLUP) {
            // Param is the callback function
            // Set the direction and turn on the pullup
            _expander.setDir(_gpio,0);
            _expander.setPullUp(_gpio,1);
            // Fall through to the callback setup
        }
        
        if (typeof param == "function") {
            // If we have a callback, configure it against a rising IRQ edge
            _expander.setIrqMask(_gpio,1);
            _expander.setIrqEdges(_gpio,1,1);
            _expander.setCallback(_gpio, param);
        } else {
            // Disable the callback for this pin
            _expander.setIrqMask(_gpio,0);
            _expander.setIrqEdges(_gpio,0,0);
            _expander.setCallback(_gpio,null);
        }
        
        return this;
    }
    
    // Reads the stats of the configured pin
    function read() { 
        return _expander.getPin(_gpio); 
    }
    
    // Sets the state of the configured pin
    function write(state) { 
        _expander.setPin(_gpio,state); 
    }
    
    // Set the intensity of an LED OUT pin. Don't use for other pin types.
    function setIntensity(intensity) { 
        _expander.setIntensityOn(_gpio,intensity); 
    }
    
    // Set the blink rate of an LED OUT pin. Don't use for other pin types.
    function blink(rampup, rampdown, intensityon, intensityoff = 0, fade=true) { 
        rampup = (rampup > 0x1F ? 0x1F : rampup);
        rampdown = (rampdown > 0x1F ? 0x1F : rampdown);
        intensityon = intensityon & 0xFF;
        intensityoff = (intensityoff > 0x07 ? 0x07 : intensityoff);
        
        _expander.setTimeOn(_gpio, rampup);
        _expander.setOff(_gpio, rampdown << 3 | intensityoff);
        _expander.setRiseTime(_gpio, fade?5:0);
        _expander.setFallTime(_gpio, fade?5:0);
        _expander.setIntensityOn(_gpio, intensityon);
        _expander.setPin(_gpio, intensityon>0 ? 0 : 1)
    }
    
    // Enable or disable fading (breathing)
    function fade(on, risetime = 5, falltime = 5) {
        _expander.setRiseTime(_gpio, on ? risetime : 0);
        _expander.setFallTime(_gpio, on ? falltime : 0);
    }
}

class RGBLED {
    
    _expander = null;
    ledR = null;
    ledG = null;
    ledB = null;
    
    // Constructor requires the IO Expander object but the three pin numbers for R, G and B
    constructor(expander, gpioRed, gpioGreen, gpioBlue) {
        _expander = expander;
        ledR = ExpGPIO(_expander, gpioRed).configure(ExpGPIO.LED_OUT);
        ledG = ExpGPIO(_expander, gpioGreen).configure(ExpGPIO.LED_OUT);
        ledB = ExpGPIO(_expander, gpioBlue).configure(ExpGPIO.LED_OUT);
    }
    
    // Returns a table with the last/current values of the R, G and B intensities
    function read() {
        return {r = (256 - ledR.read() * 256).tointeger(), 
                g = (256 - ledG.read() * 256).tointeger(), 
                b = (256 - ledB.read() * 256).tointeger()};
    }
    
    // Set the colour intensities (0-255) and an optional fade (boolean)
    function set(r, g, b, fade=false) {
        ledR.blink(0, 0, r.tointeger(), 0, fade);
        ledG.blink(0, 0, g.tointeger(), 0, fade);
        ledB.blink(0, 0, b.tointeger(), 0, fade);
    }
    
    // Blink the LEDs at the given intensity, and time with optional fading (breathing)
    function blink(r, g, b, fade=true, timeon=1, timeoff=1) {
        // Turn them off and let them sync on their way on
        ledR.write(1); ledG.write(1); ledB.write(1); 
        ledR.blink(timeon.tointeger(), timeoff.tointeger(), r.tointeger(), 0, fade);
        ledG.blink(timeon.tointeger(), timeoff.tointeger(), g.tointeger(), 0, fade);
        ledB.blink(timeon.tointeger(), timeoff.tointeger(), b.tointeger(), 0, fade);
    }
    
}

enum CAP_COLOUR { RED, GREEN, BLUE, CLEAR };

class TempSensor_rev3 {

    _i2c  = null;
    _addr = null;
    _expander = null;
    _alert = null;
    _alert_callback = null;
    _poll_callback = null;
    _poll_interval = null;
    _poll_timer = null;
    _last_temp = null;
    _running = false;
    _disabled = false;
    
    static REG_TEMP      = "\x00";
    static REG_CONF      = "\x01";
    static REG_T_LOW     = "\x02";
    static REG_T_HIGH    = "\x03";
    
    // Constructor requires the I2C and IO Expander objects and the pin number to send alerts to
    constructor(i2c, address, expander, gpioAlert) {
        _i2c  = i2c;
        _addr = address;  
        _expander = expander;
        
        // Check we have the right sensor on this address
        local id = _i2c.read(_addr, REG_TEMP, 1);
        if (id == null) {
            server.error(format(ERR_WRONG_DEVICE, _addr, "TMP112 temperature sensor"))
            _disabled = true;
        } else {
            // Setup the alert pin
            _alert = ExpGPIO(_expander, gpioAlert).configure(DIGITAL_IN_PULLUP, _interruptHandler.bindenv(this));
            
            // Shutdown the sensor for now
            local conf = _i2c.read(_addr, REG_CONF, 2);
            _i2c.write(_addr, REG_CONF + format("%c%c", conf[0] | 0x01, conf[1]));
        }
    }
    
    // Handles rising edges on the alert pin and triggers the callback
    function _interruptHandler(state) {
        if (_alert_callback && state == 0) _alert_callback(read());
    }    
    
    // Regularly report the temperature to the callback function, but only if its changed
    function poll(interval = null, callback = null) {
        if (_disabled) return server.error(format(ERR_NO_DEVICE, _addr));
        
        if (interval && callback) {
            _poll_interval = interval;
            _poll_callback = callback;
            if (_poll_timer) imp.cancelwakeup(_poll_timer);
        } else if (!_poll_interval || !_poll_callback) {
            server.error(format(ERR_BAD_TIMER, "TempSensor_rev2::poll()"))
            return false;
        }
        
        local temp = read();
        _poll_timer = imp.wakeup(_poll_interval, poll.bindenv(this))
        if (temp != _last_temp) {
            _poll_callback(temp);
            _last_temp = temp;
        }
        
    }
    
    // Setup an alert for when the temperature crosses below the lo or above the hi value.
    function alert(lo, hi, callback = null) {
        if (_disabled) return server.error(format(ERR_NO_DEVICE, _addr));
        
        callback = callback ? callback : _poll_callback;
        stop();
        _alert_callback = callback;
    
        local tlo = deg2int(lo, 0.0625, 12);
        local thi = deg2int(hi, 0.0625, 12);
        _i2c.write(_addr, REG_T_LOW + format("%c%c", (tlo >> 8) & 0xFF, (tlo & 0xFF)));
        _i2c.write(_addr, REG_T_HIGH + format("%c%c", (thi >> 8) & 0xFF, (thi & 0xFF)));
        _i2c.write(_addr, REG_CONF + "\x62\x80"); // Run continuously

        // Keep track of the fact that we are running continuously
        _running = true;       
    }
    
    // Stopps the poller and alert and powers the sensor down
    function stop() {
        if (_disabled) return server.error(format(ERR_NO_DEVICE, _addr));
        
        if (_poll_timer) imp.cancelwakeup(_poll_timer);
        _poll_timer = null;
        _poll_interval = null;
        _poll_callback = null;
        _alert_callback = null;
        _running = false;
        
        // Power the sensor down
        local conf = _i2c.read(_addr, REG_CONF, 2);
        _i2c.write(_addr, REG_CONF + format("%c%c", conf[0] | 0x01, conf[1]));
        
    }
    
    // Returns the current temperature
    function read() {
        if (_disabled) return server.error(format(ERR_NO_DEVICE, _addr));
        
        if (!_running) {
            local conf = _i2c.read(_addr, REG_CONF, 2);
            _i2c.write(_addr, REG_CONF + format("%c%c", conf[0] | 0x80, conf[1]));
        
            // Wait for conversion to be finished
            while ((_i2c.read(_addr, REG_CONF, 1)[0] & 0x80) == 0x80);
        }
        
        // Get 12-bit signed temperature value in 0.0625C steps
        local result = _i2c.read(_addr, REG_TEMP, 2);
        local temp = (result[0] << 8) + result[1];
        return int2deg(temp, 0.0625, 12);
    }
    
}

class Potentiometer {
    
    _expander = null;
    _gpioEnable = null;
    _pinRead = null;
    _poll_callback = null;
    _poll_interval = 0.2;
    _poll_timer = null;
    _last_pot_value = null;
    _min = 0.0;
    _max = 1.0;
    _integer_only = false;

    constructor(expander, gpioEnable, pinRead) {
        _expander = expander;
        _pinRead = pinRead;
        _pinRead.configure(ANALOG_IN);
        _gpioEnable = ExpGPIO(_expander, gpioEnable).configure(DIGITAL_OUT);
    }
    
    // Regularly reads the pot value and returns it to the callback when the value changes
    function poll(interval = null, callback = null) {
        if (interval && callback) {
            _poll_interval = interval;
            _poll_callback = callback;
            if (_poll_timer) imp.cancelwakeup(_poll_timer);
        } else if (!_poll_interval || !_poll_callback) {
            server.error(format(ERR_BAD_TIMER, "TempSensor_rev2::poll()"))
            return false;
        }
        
        _poll_timer = imp.wakeup(_poll_interval, poll.bindenv(this))
        local new_pot_value = read();
        if (_last_pot_value != new_pot_value) {
            _last_pot_value = new_pot_value;
            _poll_callback(new_pot_value);
        }
        
    }

    // Stops the poller
    function stop() {
        if (_poll_timer) imp.cancelwakeup(_poll_timer);
        _poll_timer = null;
        _poll_interval = null;
        _poll_callback = null;
    }

    // Enable or disable the potentiometer
    function setenabled(enable = true) {
        _gpioEnable.write(enable ? 0 : 1);
        if (_checkpot_timer) {
            imp.cancelwakeup(_checkpot_timer);
        }
        if (enable && _callback) {
            _checkpot_timer = imp.wakeup(0, checkpot.bindenv(this));
        }
    }
    
    // Get the enabled status
    function enabled() {
        return _gpioEnable.read() == 0;
    }

    // Sets the minimum and maximum of the output scale. Optionally limit the values to integers.
    function scale(min, max, integer_only = false) {
        _min = min;
        _max = max;
        _integer_only = integer_only;
    }
    
    
    // Gets the current value, rounded to an integer or three decimal places 
    function read() {
        local f = 0.0 + _min + (_pinRead.read() * (_max - _min) / 65535.0);
        if (_integer_only) return f.tointeger();
        else               return format("%0.02f", f).tofloat();
    }
    
}

class Accelerometer_rev3 {
    
    _i2c = null;
    _addr = null;
    _expander = null;
    _gpioInterrupt = null;
    _poll_timer = null;
    _poll_interval = null;
    _poll_callback = null;
    _alert_callback = null;
    _disabled = false;
    _running = false;
    
    static CTRL_REG1     = "\x20";
    static CTRL_REG2     = "\x21";
    static CTRL_REG3     = "\x22";
    static CTRL_REG4     = "\x23";
    static CTRL_REG5     = "\x24";
    static CTRL_REG6     = "\x25";
    static DATA_X_L      = "\x28";
    static DATA_X_H      = "\x29";
    static DATA_Y_L      = "\x2A";
    static DATA_Y_H      = "\x2B";
    static DATA_Z_L      = "\x2C";
    static DATA_Z_H      = "\x2D";
    static DATA_ALL      = "\xA8";
    static INT1_CFG      = "\x30";
    static INT1_SRC      = "\x31";
    static INT1_THS      = "\x32";
    static INT1_DURATION = "\x33";
    static TAP_CFG       = "\x38";
    static TAP_SRC       = "\x39";
    static TAP_THS       = "\x3A";
    static TIME_LIMIT    = "\x3B";
    static TIME_LATENCY  = "\x3C";
    static TIME_WINDOW   = "\x3D";
    static WHO_AM_I      = "\x0F";
    
    // This constructor requires the I2C and IO Expander objects, the address of the device
    // and the expander pin number for the interrupt line.
    constructor(i2c, addr, expander, gpioInterrupt)
    {
        _i2c  = i2c;
        _addr = addr;  
        _expander = expander;
        
        local id = _i2c.read(_addr, WHO_AM_I, 1);
        if (!id || id[0] != 0x33) {
            server.error(format(ERR_WRONG_DEVICE, _addr, "LIS3DH accelerometer"))
            _disabled = true;
        } else {
            _gpioInterrupt = ExpGPIO(_expander, gpioInterrupt).configure(DIGITAL_IN, _interruptHandler.bindenv(this));
            _i2c.write(_addr, CTRL_REG1 + "\x00");      // Turn off the sensor
        }
    }
    
    // Handles the edge changes on the alert pin and calls the callback
    function _interruptHandler(state) {
        if (state == 1 && _alert_callback) {
            _alert_callback(read());
        }
    }
    
    // Configures the chip to toggle the alert pin when the device moves in any direction.
    function alert(callback = null) {
        if (_disabled) return server.error(format(ERR_NO_DEVICE, _addr));
        
        _alert_callback = callback;
        _running = true;

        // Setup the accelerometer for sleep-polling
        _i2c.write(_addr, CTRL_REG1 + "\xA7");      // Turn on the sensor, enable X, Y, and Z, ODR = 100 Hz
        _i2c.write(_addr, CTRL_REG2 + "\x00");      // High-pass filter disabled
        _i2c.write(_addr, CTRL_REG3 + "\x40");      // Interrupt driven to INT1 pad
        _i2c.write(_addr, CTRL_REG4 + "\x00");      // FS = 2g
        _i2c.write(_addr, CTRL_REG5 + "\x00");      // Interrupt latched
        _i2c.write(_addr, CTRL_REG6 + "\x00");      // Interrupt Active High
        _i2c.write(_addr, INT1_THS + "\x08");       // Set movement threshold = ? mg
        _i2c.write(_addr, INT1_DURATION + "\x00");  // Duration not relevant
        _i2c.write(_addr, INT1_CFG + "\x6A");       // Configure intertia detection axis/axes - all three. Plus 6D.
        _i2c.read(_addr, INT1_SRC, 1);              // Clear any interrupts

    }
    
    // Regularly read the acceleration data and send it to the callback
    function poll(interval = null, callback = null) {
        if (_disabled) return server.error(format(ERR_NO_DEVICE, _addr));
        if (interval && callback) {
            _poll_interval = interval;
            _poll_callback = callback;
            if (_poll_timer) imp.cancelwakeup(_poll_timer);
        } else if (!_poll_interval || !_poll_callback) {
            server.error(format(ERR_BAD_TIMER, "Accelerometer_rev3::poll()"))
            return false;
        }
        
        _poll_timer = imp.wakeup(_poll_interval, poll.bindenv(this))
        _poll_callback(read());
    }
    
    // Stop the poller and the alert functionality
    function stop() {
        if (_disabled) return server.error(format(ERR_NO_DEVICE, _addr)); 
        if (_poll_timer) imp.cancelwakeup(_poll_timer);
        _poll_timer = null;
        _poll_interval = null;
        _poll_callback = null;
        _alert_callback = null;
        _running = false;
        _i2c.write(_addr, CTRL_REG1 + "\x00");      // Turn off the sensor
    }
    
    // Read the accelerometer data and return it as a table
    function read() {
        if (_disabled) return server.error(format(ERR_NO_DEVICE, _addr));

        // Configure settings of the accelerometer
        if (!_running) {
            _i2c.write(_addr, CTRL_REG1 + "\x47");  // Turn on the sensor, enable X, Y, and Z, ODR = 50 Hz
            _i2c.write(_addr, CTRL_REG2 + "\x00");  // High-pass filter disabled
            _i2c.write(_addr, CTRL_REG3 + "\x40");  // Interrupt driven to INT1 pad
            _i2c.write(_addr, CTRL_REG4 + "\x00");  // FS = 2g
            _i2c.write(_addr, CTRL_REG5 + "\x00");  // Interrupt Not latched
            _i2c.write(_addr, CTRL_REG6 + "\x00");  // Interrupt Active High (not actually used)
            _i2c.read(_addr, INT1_SRC, 1);          // Clear any interrupts
        }
        
        local data = _i2c.read(_addr, DATA_ALL, 6);
        local x = 0, y = 0, z = 0;
        if (data != null) {
            x = (data[1] << 8 | data[0]);
            if (x & 0x8000) x = -((~x & 0x7FFF) + 1);
            x = x / 32767.0;
            
            y = (data[3] << 8 | data[2]);
            if (y & 0x8000) y = -((~y & 0x7FFF) + 1);
            y = y / 32767.0;
            
            z = (data[5] << 8 | data[4]);
            if (z & 0x8000) z = -((~z & 0x7FFF) + 1);
            z = z / 32767.0;
            
            return {x = x, y = y, z = z};
        }
    }
}

class Hannah {
    
    i2c = null;
    ioexp = null;
    pot = null;
    btn1 = null;
    btn2 = null;
    hall = null;
    srv1 = null;
    srv2 = null;
    acc = null;
    led = null;
    light = null;
    temp = null;
    
    on_pot_changed = null;
    on_btn1_changed = null;
    on_btn2_changed = null;
    on_hall_changed = null;
    on_acc_changed = null;
    on_light_changed = null;
    on_temp_changed = null;
    
    constructor() {
        
        // Initialize the I2C bus
        i2c = hardware.i2c89;
        i2c.configure(CLOCK_SPEED_400_KHZ);
        
        // Initialize IO expander
        ioexp = SX1509(i2c, 0x7C, hardware.pin1);
        
        // Potentiometer on pin 2 and enabled on IO pin 8
        pot = Potentiometer(ioexp, 8, hardware.pin2);
        pot.poll(0.5 /* poll interval, sec */, call_callback("on_pot_changed"));
        
        // Button 1 on IO pin 0
        btn1 = ExpGPIO(ioexp, 0).configure(DIGITAL_IN_PULLUP, call_callback("on_btn1_changed"));
        
        // Button 2 on IO pin 1
        btn2 = ExpGPIO(ioexp, 1).configure(DIGITAL_IN_PULLUP, call_callback("on_btn2_changed"));

        // Accelerometer on i2c port 0x38 or 0x30 with alert in pin on IO pin 3
        acc = Accelerometer_rev3(i2c, 0x30, ioexp, 3);
        acc.poll(0.5, call_callback("on_acc_changed"));
        
        // RGB LED on IO pins 7 (red), 5 (green) and 6 (blue)
        led = RGBLED(ioexp, 7, 5, 6);
    }
    
    
    function call_callback(callback_name) {
        return function(a=null, b=null, c=null) {
            if ((callback_name in this) && (typeof this[callback_name] == "function")) {
                if (a == null) {
                    //this[callback_name]();
                } else if (b == null) {
                    this[callback_name](a);
                } else if (c == null) {
                    this[callback_name](a, b);
                } else {
                    this[callback_name](a, b, c);
                }
            }
        }.bindenv(this)
    }
}

function round(val, decimalPoints) {
    local f = math.pow(10, decimalPoints) * 1.0;
    local newVal = val * f;
    newVal = math.floor(newVal + 0.5)
    newVal = (newVal * 1.0) / f;
 
   return newVal;
}



//==============================================================================

hannah <- Hannah();
hannah.pot.scale(0.0, 1.0, false);

// 0..1
hannah.on_pot_changed = function(state) {
    
    if (state < 0.5) {
      hannah.led.set(0, 0, (0.5 - state) / 0.5 * 20, false);
    } else {
      hannah.led.set((state - 0.5) / 0.5 * 20, 0, 0, false);
    }
    
    local pos;
    
    // convet to -1..1
    pos = (state - 0.5) * 2;
    
    // server.log("pot: " + pos);
    agent.send("pot", pos);
}

hannah.on_btn1_changed = function(state) {
    // server.log("Button 1 is triggered: " + (state ? "up" : "down"));
    
    if (state) hannah.led.set(20, 20, 0, false);
    if (!state) {
      agent.send("claw", 0);
    }
}

hannah.on_btn2_changed = function(state) {
  // server.log("Button 2 is triggered: " + (state ? "up" : "down"));
  
  if (state) hannah.led.set(20, 0, 20, false);
  
  if (!state) {
    agent.send("claw", 1);
  }
}

hannah.on_acc_changed = function(acc) {

    local xa, ya, za;
    
    // convert into angle
    xa = math.atan(acc.x / (math.sqrt(acc.y * acc.y + acc.z * acc.z))) * 180 / PI;
    ya = math.atan(acc.y / (math.sqrt(acc.x * acc.x + acc.z * acc.z))) * 180 / PI;
    za = math.atan(math.sqrt(acc.x * acc.x + acc.y * acc.y) / acc.z) * 180 / PI;
    
    // normalize
    
    xa *= 1.07;
    za *= 1.07;

    if (xa > 90) xa = 90;
    if (xa < -90) xa = -90;
    
    if (za > 90) za = 90;
    if (za < -90) za = -90;
    
    // // convert to -1..1 range
    xa /= 90;
    xa = round(xa, 2);
    za /= 90;
    za = round(za, 2);
    
    if (za > 0) {
      // server.log(xa);
    } else {
      if (xa > 0) {
        xa = 1 + (1 - xa);
      } else {
        xa = -1 - (1 + xa);
      }
    }
    
    xa = round(xa, 2);
    
    // server.log(format("%0.1f", xa));
    agent.send("tilt", xa);
}
