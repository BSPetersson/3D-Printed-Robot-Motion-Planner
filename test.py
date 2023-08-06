def turn_motors(degrees, time_s):
    time_ms = time_s*1000000
    J1_steps = (degrees[0]/1.8)*J1_microstep
    J2_steps = (degrees[1]/1.8)*J2_microstep
    J3_steps = (degrees[2]/1.8)*J3_microstep
    J4_steps = (degrees[3]/1.8)*J4_microstep
    J5_steps = (degrees[4]/1.8)*J5_microstep
    J6_steps = (degrees[5]/1.8)*J6_microstep

    if not J1_steps == 0:
        J1_microsecond_delay = (time_ms/J1_steps)/2
        J1_microsecond_delay_now = J1_microsecond_delay
    if not J2_steps == 0:
        J2_microsecond_delay = time_ms/J2_steps
    if not J3_steps == 0:
        J3_microsecond_delay = time_ms/J3_steps
    if not J4_steps == 0:
        J4_microsecond_delay = time_ms/J4_steps
    if not J5_steps == 0:
        J5_microsecond_delay = time_ms/J5_steps
    if not J6_steps == 0:
        J6_microsecond_delay = time_ms/J6_steps

    J1_pul = True

    start_time_ms = (datetime.datetime.now() - program_start_time).microseconds
    end_time_ms = (datetime.datetime.now() - program_start_time).microseconds + (time_ms)
    while datetime.datetime.now().microsecond < end_time_ms:
        timer = datetime.datetime.now().microsecond - start_time_ms
        if timer > J1_microsecond_delay_now:
            print(start_time_ms)
            J1_pul = not J1_pul
            J1_microsecond_delay_now += J1_microsecond_delay
