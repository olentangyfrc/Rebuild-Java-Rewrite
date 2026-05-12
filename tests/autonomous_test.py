from magicbot.testing import run_test

from robot import MyRobot


def test_autonomous_actions(capsys):
    run_test(MyRobot, autonomous_mode="Test Auto 1")

    captured = capsys.readouterr()
    assert "Scoring" in captured.out
    assert "Intaking" in captured.out
