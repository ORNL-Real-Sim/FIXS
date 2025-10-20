import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

def plot_learning_curve(x, scores, figure_file):
    running_avg = np.zeros(len(scores))
    for i in range(len(running_avg)):
        running_avg[i] = np.mean(scores[max(0, i-100):(i+1)])
    plt.plot(x, running_avg)
    plt.title('Running average of previous 100 scores')
    plt.savefig(figure_file)

def vtcpmf_fuel_model(v_speed, v_accel):

    """
    VT-CPFM Fuel Model
    """

    r_f = (
        1.23 * 0.6 * 0.98 * 3.28 * (v_speed**2)
        + 9.8066 * 3152 * (1.75 / 1000) * 0.033 * v_speed
        + 9.8066 * 3152 * (1.75 / 1000) * 0.033
        + 9.8066 * 3152 * 0
    )

    power = ((r_f + 1.04 * 3152 * v_accel) / (3600 * 0.92)) * v_speed

    if power >= 0:
        return 0.00078 + 0.000019556 * power + 0.000001 * (power**2)
    else:
        return 0.00078


def process_spat_for_gen_speed(tl_light,x1,x2,x3,x4,x5,x6):
                
        if tl_light[0][0] == '2':
                                
            green_max = 24
            if x1 == None:
                t1s = 0
                t1e = t1s + green_max
                t2s = t1s + 100
                t2e = t2s + green_max
                r1s = 0
                curr_status = 'green'
            # time to green, currently red
            elif x1[0] == 'g':
                t1s = x1[1]
                t1e = t1s + green_max
                t2s = t1s + 100
                t2e = t2s + green_max
                r1s = 0
                curr_status = 'red'
            # time to red, currently green
            elif x1[0] == 'r':
                t1s = 0
                t1e = x1[1]
                t2s = t1e + (100 - green_max)
                t2e = t2s + green_max 
                r1s = x1[1]
                curr_status = 'green' 
            
        elif tl_light[0][0] == '3':

            green_max = 64
            # time to green, currently red
            if x2[0] == 'g':
                t1s = x2[1]
                t1e = t1s + green_max
                t2s = t1s + 100
                t2e = t2s + green_max
                r1s = 0
                curr_status = 'red'
            # time to red, currently green
            elif x2[0] == 'r':
                t1s = 0
                t1e = x2[1]
                t2s = t1e + (100 - green_max)
                t2e = t2s + green_max 
                r1s = x2[1]
                curr_status = 'green' 

        elif tl_light[0][0] == '10':

            green_max = 46
            # time to green, currently red
            if x3[0] == 'g':
                t1s = x3[1]
                t1e = t1s + green_max
                t2s = t1s + 100
                t2e = t2s + green_max
                r1s = 0
                curr_status = 'red'
            # time to red, currently green
            elif x3[0] == 'r':
                t1s = 0
                t1e = x3[1]
                t2s = t1e + (100 - green_max)
                t2e = t2s + green_max 
                r1s = x3[1]
                curr_status = 'green' 
        
        elif tl_light[0][0] == '8':

            green_max = 38
            # time to green, currently red
            if x4[0] == 'g':
                t1s = x4[1]
                t1e = t1s + green_max
                t2s = t1s + 100
                t2e = t2s + green_max
                r1s = 0
                curr_status = 'red'
            # time to red, currently green
            elif x4[0] == 'r':
                t1s = 0
                t1e = x4[1]
                t2s = t1e + (100 - green_max)
                t2e = t2s + green_max 
                r1s = x4[1]
                curr_status = 'green' 

        elif tl_light[0][0] == '9':

            green_max = 68    
            # time to green, currently red
            if x5[0] == 'g':
                t1s = x5[1]
                t1e = t1s + green_max
                t2s = t1s + 100
                t2e = t2s + green_max
                r1s = 0
                curr_status = 'red'
            # time to red, currently green
            elif x5[0] == 'r':
                t1s = 0
                t1e = x5[1]
                t2s = t1e + (100 - green_max)
                t2e = t2s + green_max 
                r1s = x5[1]
                curr_status = 'green' 

        elif tl_light[0][0] == '12':

            green_max = 51
            # time to green, currently red
            if x6[0] == 'g':
                t1s = x6[1]
                t1e = t1s + green_max
                t2s = t1s + 100
                t2e = t2s + green_max
                r1s = 0
                curr_status = 'red'
            # time to red, currently green
            elif x6[0] == 'r':
                t1s = 0
                t1e = x6[1]
                t2s = t1e + (100 - green_max)
                t2e = t2s + green_max 
                r1s = x6[1]
                curr_status = 'green' 

        return t1s, t1e, t2s, t2e, r1s, curr_status


def process_spat_for_gen_speed_update(tl_light, travel_direction, phase_tracking_dict, spat_statuses):

    next_tl_light = tl_light[0][0]
    # get the upcoming phase of the ego vehicle
    phase_number = [key for key, values in phase_tracking_dict[next_tl_light].tlid_phase_config.items() if values['approach_direction'] == travel_direction ][0]

    green_max = phase_tracking_dict[next_tl_light].tlid_phase_config[phase_number]['maxDur'] + phase_tracking_dict[next_tl_light].tlid_phase_config[phase_number]['yellow']

    next_phase_status = spat_statuses[next_tl_light][phase_number]

    buffer_time = 2

    # time to green, currently red
    if next_phase_status[0] == 'g':
        t1s = next_phase_status[1]
        t1e = t1s + green_max
        t2s = t1s + 100
        t2e = t2s + green_max
        r1s = 0
        curr_status = 'red'

        # add some buffer for red to green
        t1s = t1s + buffer_time
        t1e = t1e - buffer_time

    # time to red, currently green
    elif next_phase_status[0] == 'r':
        t1s = 0
        t1e = next_phase_status[1]
        t2s = t1e + (100 - green_max)
        t2e = t2s + green_max
        r1s = next_phase_status[1]
        curr_status = 'green'

        # add some buffer for red to green
        t1e = t1e - buffer_time

    return t1s, t1e, t2s, t2e, r1s, curr_status


def vtcpmf_fuel_model(v_speed, v_accel):
    """
    Virginia Tech Comprehensive Power-Based Fuel Consumption Model: Model development and testing
    Parameters are from https://www.energy.gov/sites/default/files/2021-07/VTO_2020_APR_EEMS_06172021_compliant_.pdf
    VT-CPFM Fuel Model
    v_speed: speed in km/h
    v_accel: accel in m/s2
    m is the vehicle mass (kg), a(t) is the vehicle acceleration (m/s2) at time t, v(t) is the vehicle speed (km/h) at time t

    Output: fuel consumption rate: (l/s)
    """
    r_f = (
              (1.23/25.92) * 0.3 * 1.0 * 2.32 * (v_speed ** 2)
            + 9.8066 * 1453 * (1.75 / 1000) * 0.328 * v_speed
            + 9.8066 * 1453 * (1.75 / 1000) * 4.58
            + 9.8066 * 1453 * 0
    )

    # kw
    power = ((r_f + 1.04 * 1453 * v_accel) / (3600 * 0.92)) * v_speed

    if power >= 0:
        return 0.000592 + 0.0000424 * power + 0.000001 * (power ** 2)
    else:
        return 0.000592


def get_vtcpmf_fuel_consumption(speed, accel, sim_step_duration):
    """
    fuel consumption rate: (l/s)
    """
    return vtcpmf_fuel_model(speed, accel) * sim_step_duration


def energyVTMicro(efzp_traj, vtMicroCoeff, M=1.6443):
    """
    Input is a dataframe

    speed unit is m/s, acceleration unit is m/s2
    for kinematic energy, K.E. = 1/2 mv2, one Joule is equal to 1 kg m2 / s2
    :param efzp_traj:
    :param vtMicroCoeff:
    :param sim_step_duration:
    :param M:
    :return:
    """
    # split into two dataframes based on their Acc
    efzp_traj_negative = efzp_traj[efzp_traj['acceleration'] < 0].reset_index(drop=True)
    efzp_traj_non_negative = efzp_traj[efzp_traj['acceleration'] >= 0].reset_index(drop=True)

    # calculate the instantaneous fuel consumption using VT-Micro Model (liters/sec, https://www-sciencedirect-com.ornl.idm.oclc.org/science/article/pii/S0191261514001684#b0005)
    efzp_traj_non_negative['instantFuelConsumptionVTMicro'] = np.exp(vtMicroCoeff.iloc[0, 2] +
                                                              vtMicroCoeff.iloc[0, 3] * 3.6 * efzp_traj_non_negative['acceleration'] +
                                                              vtMicroCoeff.iloc[0, 4] * np.power(3.6 * efzp_traj_non_negative['acceleration'], 2) +
                                                              vtMicroCoeff.iloc[0, 5] * np.power(3.6 * efzp_traj_non_negative['acceleration'], 3) +
                                                              vtMicroCoeff.iloc[1, 2] * 3.6 * efzp_traj_non_negative['speed'] +
                                                              vtMicroCoeff.iloc[1, 3] * 3.6 * efzp_traj_non_negative['speed'] * 3.6 * efzp_traj_non_negative['acceleration'] +
                                                              vtMicroCoeff.iloc[1, 4] * 3.6 * efzp_traj_non_negative['speed'] * np.power(3.6 * efzp_traj_non_negative['acceleration'], 2) +
                                                              vtMicroCoeff.iloc[1, 5] * 3.6 * efzp_traj_non_negative['speed'] * np.power(3.6 * efzp_traj_non_negative['acceleration'], 3) +
                                                              vtMicroCoeff.iloc[2, 2] * np.power(3.6 * efzp_traj_non_negative['speed'], 2) +
                                                              vtMicroCoeff.iloc[2, 3] * np.power(3.6 * efzp_traj_non_negative['speed'], 2) * 3.6 * efzp_traj_non_negative['acceleration'] +
                                                              vtMicroCoeff.iloc[2, 4] * np.power(3.6 * efzp_traj_non_negative['speed'], 2) * np.power(3.6 * efzp_traj_non_negative['acceleration'], 2) +
                                                              vtMicroCoeff.iloc[2, 5] * np.power(3.6 * efzp_traj_non_negative['speed'], 2) * np.power(3.6 * efzp_traj_non_negative['acceleration'], 3) +
                                                              vtMicroCoeff.iloc[3, 2] * np.power(3.6 * efzp_traj_non_negative['speed'], 3) +
                                                              vtMicroCoeff.iloc[3, 3] * np.power(3.6 * efzp_traj_non_negative['speed'], 3) * 3.6 * efzp_traj_non_negative['acceleration'] +
                                                              vtMicroCoeff.iloc[3, 4] * np.power(3.6 * efzp_traj_non_negative['speed'], 3) * np.power(3.6 * efzp_traj_non_negative['acceleration'], 2) +
                                                              vtMicroCoeff.iloc[3, 5] * np.power(3.6 * efzp_traj_non_negative['speed'], 3) * np.power(3.6 * efzp_traj_non_negative['acceleration'], 3))

    efzp_traj_negative['instantFuelConsumptionVTMicro'] = np.exp(vtMicroCoeff.iloc[4, 2] +
                                                              vtMicroCoeff.iloc[4, 3] * 3.6 * efzp_traj_negative['acceleration'] +
                                                              vtMicroCoeff.iloc[4, 4] * np.power(3.6 * efzp_traj_negative['acceleration'], 2) +
                                                              vtMicroCoeff.iloc[4, 5] * np.power(3.6 * efzp_traj_negative['acceleration'], 3) +
                                                              vtMicroCoeff.iloc[5, 2] * 3.6 * efzp_traj_negative['speed'] +
                                                              vtMicroCoeff.iloc[5, 3] * 3.6 * efzp_traj_negative['speed'] * 3.6 * efzp_traj_negative['acceleration'] +
                                                              vtMicroCoeff.iloc[5, 4] * 3.6 * efzp_traj_negative['speed'] * np.power(3.6 * efzp_traj_negative['acceleration'], 2) +
                                                              vtMicroCoeff.iloc[5, 5] * 3.6 * efzp_traj_negative['speed'] * np.power(3.6 * efzp_traj_negative['acceleration'], 3) +
                                                              vtMicroCoeff.iloc[6, 2] * np.power(3.6 * efzp_traj_negative['speed'], 2) +
                                                              vtMicroCoeff.iloc[6, 3] * np.power(3.6 * efzp_traj_negative['speed'], 2) * 3.6 * efzp_traj_negative['acceleration'] +
                                                              vtMicroCoeff.iloc[6, 4] * np.power(3.6 * efzp_traj_negative['speed'], 2) * np.power(3.6 * efzp_traj_negative['acceleration'], 2) +
                                                              vtMicroCoeff.iloc[6, 5] * np.power(3.6 * efzp_traj_negative['speed'], 2) * np.power(3.6 * efzp_traj_negative['acceleration'], 3) +
                                                              vtMicroCoeff.iloc[7, 2] * np.power(3.6 * efzp_traj_negative['speed'], 3) +
                                                              vtMicroCoeff.iloc[7, 3] * np.power(3.6 * efzp_traj_negative['speed'], 3) * 3.6 * efzp_traj_negative['acceleration'] +
                                                              vtMicroCoeff.iloc[7, 4] * np.power(3.6 * efzp_traj_negative['speed'], 3) * np.power(3.6 * efzp_traj_negative['acceleration'], 2) +
                                                              vtMicroCoeff.iloc[7, 5] * np.power(3.6 * efzp_traj_negative['speed'], 3) * np.power(3.6 * efzp_traj_negative['acceleration'], 3))

    # Combine the negative and non-negative dataframe
    efzp_traj = pd.concat([efzp_traj_non_negative, efzp_traj_negative], ignore_index=True)

    efzp_traj = efzp_traj.sort_values(by=['id', 'time']).reset_index(drop=True)
    #
    return efzp_traj


def cal_trac_energy(efzp_traj, df_vehicle_src_coeff, regenerative_coef=0.7, veh_type=13):
    row = df_vehicle_src_coeff[df_vehicle_src_coeff['VehicleType'] == veh_type]
    # Calculate the VSP/STP (assuming roadway grade is zero)

    A = row['A'].values
    B = row['B'].values
    C = row['C'].values
    M = row['M'].values

    efzp_traj['trac_power'] = A * efzp_traj['speed'] + \
                              B * np.power(efzp_traj['speed'], 2) + \
                              C * np.power(efzp_traj['speed'], 3) + M * efzp_traj['acceleration'] * efzp_traj['speed']

    # energy evaluation considering regenerative braking
    efzp_traj['trac_power_regen'] = np.where(efzp_traj['trac_power'] < 0, efzp_traj['trac_power'] * regenerative_coef, efzp_traj['trac_power'])

    efzp_traj['trac_power'] = np.where(efzp_traj['trac_power'] < 0, 0, efzp_traj['trac_power'])

    return efzp_traj


def getICVenergy(v, a):
    # From Yunli
    # Input:
    #   v: speed in m/s
    #   a: acceleration in m/s^2
    # Output:
    #   fuel: fuel consumption in liter per second

    p00 = 0.4415
    p10 = 0.02964
    p01 = 0.04654
    p20 = -0.000134
    p11 = 0.07274

    idleFuelGramPerSecond = 0.318

    fuelGramPerSecond = max(p00 + p10 * v + p01 * a + p20 * v ** 2 + p11 * v * a, idleFuelGramPerSecond)

    fuelLiterPerSecond = fuelGramPerSecond * 0.00133529

    return fuelLiterPerSecond


def getEVenergy(v, a):
    # From Yunli
    # Input:
    #   v: speed in m/s
    #   a: acceleration in m/s^2
    # Output:
    #   batteryPowerKw: battery power in kW
    #
    #   after getting power of each (v,a), multiply all battery power with delta time: batteryTotalKwh=sum(batteryPowerKw*dt/3600)
    #   after getting batteryTotalKwh, calculate equivalent gallon as batteryGallonEquivalent = batteryTotalKwh/33.7
    #   then can calculate MPGe using MPGe=distanceTotalMile/batteryGallonEquivalent

    p00 = 6.3331
    p10 = 0.6529
    p01 = 0.5910
    p20 = 0.0239
    p11 = 1.8354

    batteryPowerKw = p00 + p10 * v + p01 * a + p20 * v ** 2 + p11 * v * a

    return batteryPowerKw