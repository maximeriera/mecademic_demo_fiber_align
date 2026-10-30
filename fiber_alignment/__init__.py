"""Fiber alignment demo: the process shared by prod.py and manual_actions.py.

    poses.py      robot program names, tool frame, home/shipment joints
    cell.py       device names, ProcessError, robot setup, run_program()
    gripper.py    SmarAct gripper: open/close moves of its MCS2 jaw channel
    process.py    the process steps, plus run_cycle() and run_align_only_cycle()
    alignment.py  the contract with the alignment algorithm: 3D moves, scans, readings
    algorithm.py  the alignment algorithm: acquire, centre, tilt, peak
    memory.py     where each fiber aligned last, so that its next alignment starts there
    runlog.py     a CSV row per alignment-only run (offset, phases, result)
    fit.py       its curve fits (Gaussian profiles, log-power parabolas)
    frames.py     frame math, from the colleague's script
"""
