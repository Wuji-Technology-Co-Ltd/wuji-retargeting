from stardust_wuji_quest3_pc_retargeting.safety.state_machine import TeleopState, TeleopStateMachine


def test_state_machine_nominal_lifecycle_and_estop():
    sm = TeleopStateMachine()

    assert sm.state is TeleopState.IDLE
    sm.arm()
    assert sm.state is TeleopState.ARMED
    sm.start()
    assert sm.state is TeleopState.RUNNING
    sm.pause()
    assert sm.state is TeleopState.PAUSED
    sm.resume()
    assert sm.state is TeleopState.RUNNING
    sm.estop()
    assert sm.state is TeleopState.ESTOP
    sm.reset()
    assert sm.state is TeleopState.IDLE


def test_invalid_start_enters_fault():
    sm = TeleopStateMachine()

    sm.start()

    assert sm.state is TeleopState.FAULT
    assert "start requires ARMED" in sm.fault_reason
