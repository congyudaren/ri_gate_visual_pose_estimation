from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_new_message_files_exist() -> None:
    msg_dir = PACKAGE_ROOT / "msg"
    expected = {
        "StructureROI.msg",
        "FrontFaceROI.msg",
        "FrontFaceROIArray.msg",
        "FrontFaceCorners.msg",
        "FrontFaceDebug.msg",
    }

    assert expected.issubset({path.name for path in msg_dir.glob("*.msg")})


def test_cmake_generates_new_interfaces() -> None:
    text = (PACKAGE_ROOT / "CMakeLists.txt").read_text(encoding="utf-8")
    for name in (
        "StructureROI.msg",
        "FrontFaceROI.msg",
        "FrontFaceROIArray.msg",
        "FrontFaceCorners.msg",
        "FrontFaceDebug.msg",
    ):
        assert f'"msg/{name}"' in text
    assert "geometry_msgs" in text


def test_front_face_debug_exposes_structure_state_estimates() -> None:
    text = (PACKAGE_ROOT / "msg" / "FrontFaceDebug.msg").read_text(encoding="utf-8")

    for field in (
        "bool left_post_valid",
        "float32 left_post_x",
        "float32 left_post_z",
        "float32 left_post_confidence",
        "bool right_post_valid",
        "float32 right_post_x",
        "float32 right_post_z",
        "float32 right_post_confidence",
        "bool top_beam_valid",
        "float32 top_beam_y",
        "float32 top_beam_confidence",
    ):
        assert field in text
