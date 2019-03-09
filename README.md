# drq_ros

## Description

ROS package for interfacing with the PMC on a murata drq1250.

## Build Instructions 

* catkin_make
* make sure devel is sourced for custom drq message

## Nodes

### drq_1250

file: drq1250_ros

Node name:
* drq_1250

Topics:

* `status`:
  Publishes `drq1250/DRQ1250` status data from the DRQ. 

Services:

**Setters**

* `set_vin_uv_limit`
  Call `SetValue` - Sets DRQ voltage in undervoltage value in volts
  
* `set_vin_ov_limit`
  Call `SetValue` - Sets DRQ voltage in overvoltage value in volts
  
* `set_vout_uv_limit`
  Call `SetValue` - Sets DRQ voltage out undervoltage value in volts
  
* `set_vout_ov_limit`
  Call `SetValue` - Sets DRQ value in volts
  
* `set_iout_oc_limit`
  Call `SetValue` - Sets DRQ value in volts
  
* `set_ot_limit_handle`
  Call `SetValue` - Sets maximum overtemperature limit
  
* `set_iout_fault_response`
  Call `SetByte` - Sets fault response for iout
  
* `set_fault_response`
  Call `SetByte` - Sets general fault response
  
* `set_ton_delay`
  Call `SetValue` - Sets delay for turning on (MS)
  
* `set_ton_rise`
  Call `SetValue` - Sets rise value for turning on (MS)
  
* `set_toff_delay`
  Call `SetValue` - Sets delay for turning off (MS)
  
* `set_toff_fall`
  Call `SetValue` - Set fall for turning off (MS)
  
* `store_user_all`
  Call `Trigger` - Stores all user settings
  
* `restore_user_all`
  Call `Trigger` - Restores all user settings

* `restore_default_all`
  Call `Trigger` - Restores default user settings
  
* `clear_faults`
  Call `Trigger` - Clears all faults
  
* `reg_off`
  Call `Trigger` - Turns off regulator output
  
* `reg_on`
  Call `Trigger` - Turns on regulator output

**Getters**

 * `get_vin_uv_limit_handle`
  Call `Float`
  
* `get_vin_ov_limit_handle`
  Call `Float`
  
* `get_vout_ov_limit_handle`
  Call `Float`
  
* `get_iout_oc_limit_handle`
  Call `Float`

* `get_ot_limit_handle`
  Call `Float`
  
* `get_ton_delay_handle`
  Call `Float`
  
* `get_ton_rise_handle`
  Call `Float`
  
* `get_toff_delay_handle`
  Call `Float`
  
* `get_toff_fall_handle`
  Call `Float`
  
* `get_iout_fault_response_handle`
  Call `Byte`
  
* `get_fault_response_handle`
  Call `Byte`
  

## MSG

### DRQ1250

* `float64 Vin` - Voltage In
* `float64 Vout` - Voltage Out
* `float64 Iout` - Current Out
* `float64 Pout` - Power Out
* `float64 tempurature` - DRQ Temperature
* `float64 dutyCycle` - 
* `float64 switchingFreq` - 
* `bool busy` - 
* `bool off` - 
* `bool vout_ov_fault` - Overvoltage out fault
* `bool iout_oc_fault` - Overcurrent fault out
* `bool vin_uv_fault`- Undercurrent fault in
* `bool temp_fault` - Temperature fault
* `bool cml_fault`
* `bool vout_fault`
* `bool iout_fault`
* `bool input_fault`
* `bool pwr_gd`
* `bool fan_fault`
* `bool other`
* `bool unknown`

## Launch Information
 
NA

## Troubleshooting

## Contributors 

* Current maintaner: Michael Equi

* Contributors:
  * Michael Equi - initial work
  * Caelin Sutch - Documentation

## Helpful Resources

* Previous vector drive implementation: https://github.com/JHSRobo/Programming/blob/Development/Mako/Bottomside/bottomside/VectorDrive.py
