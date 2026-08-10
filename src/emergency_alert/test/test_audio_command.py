"""시스템 스피커 재생 명령을 만드는 규칙에 대한 단위시험."""

import pytest

from emergency_alert.audio_output import SystemSpeakerOutput


def build(player, device=""):
    """플레이어와 장치만 지정한 SystemSpeakerOutput을 만든다.

    __init__은 실제로 설치된 플레이어를 탐색하므로 우회하고 필드만 채운다.
    명령 조립 규칙만 검사하는 시험이라 재생 스레드는 필요하지 않다.
    """
    output = SystemSpeakerOutput.__new__(SystemSpeakerOutput)
    output._player = player
    output._device = device
    return output


def test_paplay_reads_stdin_without_a_file_argument():
    """paplay는 "-"를 파일 이름으로 취급해 실패하므로 인자를 붙이지 않는다.

    이 한 글자 때문에 합성 톤인 출동·도착·중단 경보가 소리 없이 모두
    실패했다. paplay는 파일 인자가 없을 때 stdin에서 읽는다.
    """
    assert build("paplay")._command("") == ["paplay"]
    assert "-" not in build("paplay", "bluez_sink.X")._command("")


def test_other_players_keep_the_stdin_dash():
    """aplay와 pw-play는 "-"를 stdin으로 해석하므로 그대로 둔다."""
    assert build("aplay")._command("") == ["aplay", "-"]
    assert build("pw-play")._command("") == ["pw-play", "-"]


def test_file_playback_passes_the_path_for_every_player():
    """파일 재생 경로는 플레이어 종류와 무관하게 그대로 전달한다."""
    for player in ("paplay", "pw-play", "aplay"):
        assert build(player)._command("/tmp/alarm.wav")[-1] == "/tmp/alarm.wav"


@pytest.mark.parametrize(
    "player,expected",
    [
        ("paplay", "--device=bluez_sink.X"),
        ("pw-play", "--target=bluez_sink.X"),
    ],
)
def test_device_option_matches_the_player(player, expected):
    """로봇별 블루투스 스피커 지정이 플레이어마다 다른 옵션으로 나간다."""
    assert expected in build(player, "bluez_sink.X")._command("")


def test_aplay_uses_a_separate_device_flag():
    """aplay만 -D 를 값과 분리해 받는다."""
    assert build("aplay", "hw:0,0")._command("")[:3] == ["aplay", "-D", "hw:0,0"]
