#!/usr/bin/env python

import rospy
from drq1250.msg import DRQ1250
from std_srvs.srv import SetBool, Trigger
from drq1250.srv import SetValue, SetByte, Float, Byte
from pmbus import PMBus
import sys

rospy.init_node("drq2150")

myargv = rospy.myargv(sys.argv)
if len(myargv) >= 2:
    if myargv[1].startswith("0x"):
        addr = int(myargv[1][2:], 16)
    else:
        addr = int(myargv[1], 16)
else:
    rospy.logerr("USAGE WARNING: Please provide an address for the DRQ device! Using default 0x12")
    addr = 0x12

rospy.loginfo("Initializing PMBUS... ")

DRQ = PMBus(addr) #New pmbus object with device address

#Setup services to adjust and read DRQ1250 settings
def set_vin_uv_limit_handle(value):
    if value.data > 32 and value.data < 75:
        try:
            DRQ.setVinUVLimit(value.data)
        except Excetion as e:
            return False, str(e)
    else:
        return False, "Value out of bounds, should be between 32 and 75 volts!"
    return True, "VIN UV Limit set to: " +  str(DRQ.getVinUVLimit())
rospy.Service("set_vin_uv_limit", SetValue, set_vin_uv_limit_handle)

def set_vin_ov_limit_handle(value):
    if value.data > 32 and value.data < 110:
        try:
            DRQ.setVinOVLimit(value.data)
        except Excetion as e:
            return False, str(e)
    else:
        return False, "Value out of bounds, should be between 32 and 110 volts!"
    return True, "VIN OV Limit set to: " + str(DRQ.getVinOVLimit())
rospy.Service("set_vin_ov_limit", SetValue, set_vin_ov_limit_handle)

def set_vout_ov_limit_handle(value):
    if value.data > 8.1 and value.data < 15.6:
        try:
            DRQ.setVoutOVLimit(value.data)
        except Excetion as e:
            return False, str(e)
    else:
        return False, "Value out of bounds, should be between 8.1 and 15.6 volts!"
    return True, "VOUT OV Limit set to: " + str(DRQ.getVoutOVLimit())
rospy.Service("set_vout_ov_limit", SetValue, set_vout_ov_limit_handle)

def set_iout_oc_limit_handle(value):
    if value.data > 59 and value.data < 65:
        try:
            DRQ.setIoutOCLimit(value.data)
        except Excetion as e:
            return False, str(e)
    else:
        return False, "Value out of bounds, should be between 59 and 65 amps!"
    return True, "IOUT OC Limit set to: " + str(DRQ.getIoutOCLimit())
rospy.Service("set_iout_oc_limit", SetValue, set_iout_oc_limit_handle)

def set_ot_limit_handle(value):
    if value.data > 30 and value.data < 145:
        try:
            DRQ.setOTLimit(value.data)
        except Excetion as e:
            return False, str(e)
    else:
        return False, "Value out of bounds, should be between 30 and 145 degrees C!"
    return True, "OT Limit set to: " + str(DRQ.getOTLimit())
rospy.Service("set_ot_limit", SetValue, set_ot_limit_handle)

def set_iout_fault_response_handle(value):
    try:
        DRQ.setIoutFaultResponse(value.data)
    except Exception as e:
        return False, str(e)
    return True, "IOUT Fault Response set to: " + bin(DRQ.getIoutFaultResponse())
rospy.Service("set_iout_fault_response", SetByte, set_iout_fault_response_handle)

def set_fault_response_handle(value):
    try:
        DRQ.setFaultResponse(value.register, value.data)
    except Exception as e:
        return False, str(e)
    return True, "Fault Response for " + hex(value.register) + " set to: " + bin(DRQ.getFaultResponse())
rospy.Service("set_fault_response", SetByte, set_iout_fault_response_handle)

def set_ton_delay_handle(value):
    if value.data > 1 and value.data < 500:
        try:
            DRQ.setTonDelay(value.data)
        except Excetion as e:
            return False, str(e)
    else:
        return False, "Value out of bounds, should be between 1 and 500ms"
    return True, "Ton Delay set to: " + str(DRQ.getTonDelay())
rospy.Service("set_ton_delay", SetValue, set_ton_delay_handle)

def set_ton_rise_handle(value):
    if value.data > 10 and value.data < 100:
        try:
            DRQ.setTonRise(value.data)
        except Excetion as e:
            return False, str(e)
    else:
        return False, "Value out of bounds, should be between 10 and 100ms"
    return True, "Ton Rise set to: " + str(DRQ.getTonRise())
rospy.Service("set_ton_rise", SetValue, set_ton_rise_handle)

def set_toff_delay_handle(value):
    if value.data > 0 and value.data < 500:
        try:
            DRQ.setToffDelay(value.data)
        except Excetion as e:
            return False, str(e)
    else:
        return False, "Value out of bounds, should be between 0 and 500ms"
    return True, "Toff Delay set to: " + str(DRQ.getToffDelay())
rospy.Service("set_toff_delay", SetValue, set_toff_delay_handle)

def set_toff_fall_handle(value):
    if value.data > 10 and value.data < 100:
        try:
            DRQ.setToffFall(value.data)
        except Excetion as e:
            return False, str(e)
    else:
        return False, "Value out of bounds, should be between 10 and 100ms"
    return True, "Toff Fall set to: " + str(DRQ.getToffFall())
rospy.Service("set_toff_fall", SetValue, set_toff_fall_handle)

def store_user_all_handle(value):
    try:
        DRQ.storeUserAll()
    except Exception as e:
        return False, str(e)
    return True, "User settings stored!"
rospy.Service("store_user_all", Trigger, store_user_all_handle)

def restore_user_all_handle(value):
    try:
        DRQ.restoreUserAll()
    except Exception as e:
        return False, str(e)
    return True, "User settings restored!"
rospy.Service("restore_user_all", Trigger, restore_user_all_handle)

def restore_default_all_handle(value):
    try:
        DRQ.restoreDefaultAll()
    except Exception as e:
        return False, str(e)
    return True, "Default settings restored!"
rospy.Service("restore_default_all", Trigger, restore_default_all_handle)

def clear_faults_handle(value):
    try:
        DRQ.clearFaults()
    except Exception as e:
        return False, str(e)
    return True, "Faults cleared!"
rospy.Service("clear_faults", Trigger, clear_faults_handle)

def reg_off_handle(value):
    try:
        DRQ.regOff()
    except Exception as e:
        return False, str(e)
    return True, "Regulator Output OFF!"
rospy.Service("reg_off", Trigger, reg_off_handle)

def reg_on_handle(value):
    try:
        DRQ.regOn()
    except Exception as e:
        return False, str(e)
    return True, "Regulator Output ON!"
rospy.Service("reg_on", Trigger, reg_on_handle)

def get_vin_uv_limit_handle(value):
    return DRQ.getVinUVLimit()
rospy.Service("get_vin_uv_limit", Float, get_vin_uv_limit_handle)

def get_vin_ov_limit_handle(value):
    return DRQ.getVinOVLimit()
rospy.Service("get_vin_ov_limit", Float, get_vin_ov_limit_handle)

def get_vout_ov_limit_handle(value):
    return DRQ.getVouOVLimit()
rospy.Service("get_vout_ov_limit", Float, get_vout_ov_limit_handle)

def get_iot_oc_limit_handle(value):
    return DRQ.getIoutOCLimit()
rospy.Service("get_iot_oc_limit", Float, get_iot_oc_limit_handle)

def get_ot_limit_handle(value):
    return DRQ.getOTLimit()
rospy.Service("get_ot_limit", Float, get_ot_limit_handle)

def get_ton_delay_handle(value):
    return DRQ.getTonDelay()
rospy.Service("get_ton_delay", Float, get_ton_delay_handle)

def get_ton_rise_handle(value):
    return DRQ.getTonRise()
rospy.Service("get_ton_rise", Float, get_ton_rise_handle)

def get_toff_delay_handle(value):
    return DRQ.getToffDelay()
rospy.Service("get_toff_delay", Float, get_toff_delay_handle)

def get_toff_fall_handle(value):
    return DRQ.getToffFall()
rospy.Service("get_toff_fall", Float, get_toff_fall_handle)

def get_iout_fault_response_handle(value):
    return DRQ.getIoutFaultResponse()
rospy.Service("get_iout_fault_response", Byte, get_iout_fault_response_handle)

def get_fault_response_handle(value):
    return DRQ.getFaultResponse(value.register)
rospy.Service("get_fault_response", Byte, get_fault_response_handle)

pub = rospy.Publisher("status", DRQ1250, queue_size=5)
rate = rospy.Rate(30)

while not rospy.is_shutdown():
    msg = DRQ1250()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = 'drq1250'
    msg.tempurature = DRQ.getTempurature()
    msg.Vin = DRQ.getVoltageIn()
    msg.Vout = DRQ.getVoltageOut()
    msg.Iout = DRQ.getCurrent()
    msg.Pout = DRQ.getPowerOut(False) #False is caclulated from given values of current and voltage while True gets values from DRQ1250
    msg.dutyCycle = DRQ.getDutyCycle()
    msg.switchingFreq = DRQ.getSwitchingFreq()

    #handle status indicators
    status, _ =  DRQ.getStatusSummary()
    msg.busy          = status["busy"]
    msg.off           = status["off"]
    msg.vout_ov_fault = status["vout_ov_fault"]
    msg.iout_oc_fault = status["iout_oc_fault"]
    msg.vin_uv_fault  = status["vin_uv_fault"]
    msg.temp_fault    = status["temp_fault"]
    msg.cml_fault     = status["cml_fault"]
    msg.vout_fault    = status["vout_fault"]
    msg.iout_fault    = status["iout_fault"]
    msg.pout_fault    = status["pout_fault"]
    msg.input_fault   = status["input_fault"]
    msg.pwr_gd        = status["pwr_gd"]
    msg.fan_fault     = status["fan_fault"]
    msg.other         = status["other"]
    msg.unknown       = status["unknown"]

    pub.publish(msg)
    rate.sleep()
