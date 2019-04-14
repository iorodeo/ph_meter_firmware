import time
import math
import json
import board
import busio 
import analogio
import digitalio
import microcontroller
import adafruit_ht16k33.segments


class PhMeter(object):

    def __init__(self,param):
        self.param = param
        self.t_last = time.monotonic()
        self.t_last_print = self.t_last

        self.display = Display(param['show_text_dt'])
        self.display.clear()

        self.volt_sampler = VoltSampler()
        self.lowpass_filter = LowpassFilter(param['lowpass_cutoff'])

        self.calibration = PhCalibration(param['default_pH_cal'], param['pH4_pH10_margin'])
        self.display.show_text('CAL',delay=True)
        if self.calibration.is_default:
            self.display.show_text('DFL',delay=True)
        else:
            self.display.show_text('USR',delay=True)

        self.input_handler = InputHandler(param['button_hold_dt'])
        self.input_handler.register_callback( 
                {'pH7': True}, 
                {'pH7': True, 'pH4_pH10': False}, 
                self.on_calibrate_pH7
                )
        self.input_handler.register_callback(
                {'pH7': True},
                {'pH7': True, 'pH4_pH10': True},
                self.on_clear_calibration
                )
        self.input_handler.register_callback(
                {'pH4_pH10': True},
                {'pH4_pH10': True},
                self.on_calibrate_pH4_pH10
                )

    @property
    def dt(self):
        t_now = time.monotonic()
        dt = t_now - self.t_last
        self.t_last = t_now
        return dt

    @property
    def pH(self):
        volt = self.volt_sampler.median(self.param['pH_num_sample'])
        return self.calibration.volt_to_pH(volt)

    def on_calibrate_pH7(self): 
        self.display.show_text('CAL',delay=True)
        self.display.show_text('PH7',delay=True)
        volt = self.volt_sampler.median(self.param['cal_num_sample'])
        self.calibration.set_sample_pH7(volt)
        self.lowpass_filter.reset()
        self.lowpass_filter.update(self.calibration.volt_to_pH(volt),0)

    def on_calibrate_pH4_pH10(self):
        self.display.show_text('CAL',delay=True)
        volt = self.volt_sampler.median(self.param['cal_num_sample'])
        if self.calibration.ok_for_pH4(volt):
            self.display.show_text('PH4',delay=True)
            self.calibration.set_sample_pH4(volt)
        elif self.calibration.ok_for_pH10(volt):
            self.display.show_text('PH10',delay=True)
            self.calibration.set_sample_pH10(volt)
        else:
            self.display.show_text('ERR',delay=True)
        self.lowpass_filter.reset()
        self.lowpass_filter.update(self.calibration.volt_to_pH(volt),0)

    def on_clear_calibration(self): 
        self.display.show_text('----',delay=True)
        self.display.show_text('CLR', delay=True)
        self.display.show_text('CAL', delay=True)
        self.calibration.clear()

    def run(self):
        while True:
            self.input_handler.check()
            self.lowpass_filter.update(self.pH,self.dt)
            self.display.show_text('{0:0.2f}'.format(self.lowpass_filter.value))
            if self.param['stream_pH_data']: 
                now = time.monotonic()
                if (now - self.t_last_print > self.param['stream_pH_dt']):
                    print('{0:0.3f}'.format(self.lowpass_filter.value))
                    self.t_last_print = now 


class InputHandler(object):

    def __init__(self,button_hold_dt=1.0):

        self.button_hold_dt = button_hold_dt

        self.button_pH7 = digitalio.DigitalInOut(board.D12)
        self.button_pH7.direction = digitalio.Direction.INPUT
        self.button_pH7.pull = digitalio.Pull.UP

        self.button_pH4_pH10 = digitalio.DigitalInOut(board.D13)
        self.button_pH4_pH10.direction = digitalio.Direction.INPUT
        self.button_pH4_pH10.pull = digitalio.Pull.UP

        self.buttons = {'pH7': self.button_pH7,'pH4_pH10': self.button_pH4_pH10}
        self.callback_list = []
    
    def register_callback(self, state0, state1, callback):
        self.callback_list.append((state0, state1, callback))

    def check(self):
        for state0, state1, callback in self.callback_list:
            test0 = True 
            for name, value in state0.items():
                if self.buttons[name].value == value:
                    test0 = False
            if not test0:
                continue
            time.sleep(self.button_hold_dt)
            test1 = True
            for name, value in state1.items():
                if self.buttons[name].value == value:
                    test1 = False
            if not test1:
                continue
            callback()


class PhCalibration(object):

    def __init__(self, default_pH_cal, pH4_pH10_margin):
        self.default_samples = default_pH_cal
        self.pH4_pH10_margin = pH4_pH10_margin
        self.persistent_data = PersistentData()
        if not self.load_from_nvm():
            self.set_to_default()

    def set_to_default(self):
        self.user_samples = {}

    def clear(self):
        self.set_to_default()
        self.clear_from_nvm()

    @property
    def is_default(self):
        return not self.user_samples

    @property
    def slope(self):
        if set(('pH4', 'pH7')) == set(self.user_samples):
            value = 3.0/(self.volt_pH7 - self.volt_pH4)
        elif set(('pH7', 'pH10')) == set(self.user_samples):
            value = 3.0/(self.volt_pH10 - self.volt_pH7)
        else:
            value = 6.0/(self.volt_pH10 - self.volt_pH4)
        return value

    @property
    def volt_pH4(self):
        if 'pH4' in self.user_samples:
            value = self.user_samples['pH4']
        elif set(('pH7', 'pH10')) <= set(self.user_samples):
            delta_volt = self.user_samples['pH10'] - self.user_samples['pH7']
            value = self.user_samples['pH7'] - delta_volt
        else:
            value = self.default_samples['pH4']
        return value 

    @property
    def volt_pH7(self):
        if 'pH7' in self.user_samples:
            value = self.user_samples['pH7']
        elif set(('pH4', 'pH10')) <= set(self.user_samples):
            value = 0.5*(self.user_samples['pH4'] + self.user_samples['pH10'])
        else:
            value = self.default_samples['pH7']
        return value

    @property
    def volt_pH10(self):
        if 'pH10' in self.user_samples:
            value = self.user_samples['pH10']
        elif set(('pH4', 'pH7')) <= set(self.user_samples):
            delta_volt = self.user_samples['pH7'] - self.user_samples['pH4']
            value = self.user_samples['pH7'] + delta_volt
        else:
            value = self.default_samples['pH10']
        return value 

    def ok_for_pH4(self,volt):
        return volt > (self.volt_pH7 + self.pH4_pH10_margin) 

    def ok_for_pH10(self,volt):
        return volt < (self.volt_pH7 - self.pH4_pH10_margin)

    def set_sample(self,pH_str,volt):
        self.user_samples[pH_str] = volt
        self.save_to_nvm()

    def set_sample_pH4(self,volt):
        self.set_sample('pH4',volt)

    def set_sample_pH7(self,volt):
        self.set_sample('pH7',volt)

    def set_sample_pH10(self,volt):
        self.set_sample('pH10',volt)

    def volt_to_pH(self,volt):
        return self.slope*(volt - self.volt_pH7) + 7.0

    def pH_to_volt(self,pH):
        return (1.0/self.slope)*(pH - 7.0) + self.volt_pH7

    def save_to_nvm(self):
        self.persistent_data.set(self.user_samples)

    def load_from_nvm(self):
        user_samples_tmp = self.persistent_data.get()
        if user_samples_tmp is not None:
            self.user_samples = user_samples_tmp
            return True
        else:
            return False

    def clear_from_nvm(self):
        self.persistent_data.clear()


class PersistentData(object):

    def __init__(self, size=200, start_pos=0):
        self.size = size 
        self.start_pos = start_pos

    @property
    def end_pos(self):
        return self.start_pos + self.size + 1

    @property
    def is_set(self):
        return not bool(microcontroller.nvm[self.start_pos])

    def set(self,data):
        data_json = json.dumps(data)
        data_array = bytearray(data_json)
        if len(data_array) < self.size:
            n = self.size - len(data_array)
            data_array.extend(' '*n)
        microcontroller.nvm[self.start_pos] = 0
        microcontroller.nvm[(self.start_pos+1):self.end_pos] = data_array

    def get(self):
        if not self.is_set:
            return None
        data_array = microcontroller.nvm[(self.start_pos+1):self.end_pos]
        data_bytes = bytes(data_array)
        data = json.loads(data_bytes)
        return data

    def clear(self):
        microcontroller.nvm[self.start_pos:self.end_pos] = bytearray(b'\xff'*(self.size+1))



class Display(object):

    def __init__(self,show_text_dt=1.0):
        self.show_text_dt = show_text_dt
        i2c = busio.I2C(board.SCL, board.SDA)
        self.dev = adafruit_ht16k33.segments.Seg14x4(i2c, auto_write=False)
        self.dev.blink_rate = 0
        self.clear()

    def clear(self):
        self.dev.fill(0)
        self.dev.show()

    def show_text(self,text,delay=False):
        self.dev.fill(0)
        self.dev.print(text)
        self.dev.show()
        if delay:
            time.sleep(self.show_text_dt)   


class VoltSampler(object):

    def __init__(self):
        self.pH_ain = analogio.AnalogIn(board.A2)
        self.conversion_constant = self.pH_ain.reference_voltage/65535

    @property
    def volt(self):
        v = self.pH_ain.value*self.conversion_constant
        return v

    def median(self,num):
        volt_list = [self.volt for i in range(num)]
        volt_list.sort()
        if num%2 == 0:
            volt = 0.5*(volt_list[num//2 -1] + volt_list[num//2])
        else:
            volt = volt_list[num//2]
        return volt


class LowpassFilter(object):

    def __init__(self, fcut):
        self.fcut = fcut
        self.is_first = True
        self.value = 0.0

    @property
    def time_constant(self):
        rc = 1.0/(2.0*math.pi*self.fcut)
        return rc

    def update(self,x,dt):
        if self.is_first:
            self.value = x
            self.is_first = False
        else:
            coeff_0 = dt/(self.time_constant + dt)
            coeff_1 = 1.0 - coeff_0 
            self.value = coeff_0*x + coeff_1*self.value

    def reset(self):
        self.is_first = True



# -------------------------------------------------------------------------------------------------
if __name__ == '__main__':

    param = {
            'pH_num_sample'   : 100,
            'cal_num_sample'  : 5000,
            'lowpass_cutoff'  : 0.5,
            'show_text_dt'    : 1.0,
            'button_hold_dt'  : 1.0,
            'default_pH_cal'  : {'pH7': 1.65, 'pH4': 1.827, 'pH10': 1.473},
            'pH4_pH10_margin' : 0.02,
            'stream_pH_data'  : False,
            'stream_pH_dt'    : 1.0,
            }

    sensor = PhMeter(param)
    sensor.run()

