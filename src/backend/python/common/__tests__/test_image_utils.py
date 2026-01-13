import numpy as np
import pytest

from backend.generated.proto.python.sensor.camera_sensor_pb2 import ImageFormat
from backend.python.common.camera.image_utils import (
    decode_image,
    encode_image,
    from_proto_format,
    from_proto_format_to_n_channels,
    from_proto_image,
)


def test_encode_decode_roundtrip_uncompressed_bgr_exact():
    img = np.random.default_rng(0).integers(0, 256, size=(16, 20, 3), dtype=np.uint8)
    proto = encode_image(img, ImageFormat.BGR, do_compress=False)
    out = decode_image(proto)
    assert out.shape == (16, 20, 3)
    assert out.dtype == np.uint8
    assert np.array_equal(out, img)


def test_encode_decode_roundtrip_uncompressed_gray_matches_after_squeeze():
    img = np.random.default_rng(1).integers(0, 256, size=(10, 12), dtype=np.uint8)
    proto = encode_image(img, ImageFormat.GRAY, do_compress=False)
    out = decode_image(proto)
    assert out.shape == (10, 12, 1)
    assert np.array_equal(out[:, :, 0], img)


def test_encode_decode_roundtrip_compressed_shape_and_reasonable_similarity():
    img = np.random.default_rng(2).integers(0, 256, size=(32, 40, 3), dtype=np.uint8)
    proto = encode_image(img, ImageFormat.BGR, do_compress=True, compression_quality=90)
    out = decode_image(proto)
    assert out.shape == (32, 40, 3)
    # JPEG is lossy; just ensure it's not wildly different.
    assert float(np.mean(np.abs(out.astype(np.int16) - img.astype(np.int16)))) < 80.0


def test_from_proto_image_parses_serialized_imagedata():
    img = np.zeros((8, 9, 3), dtype=np.uint8)
    proto = encode_image(img, ImageFormat.BGR, do_compress=False)
    out = from_proto_image(proto.SerializeToString())
    assert out.shape == (8, 9, 3)


def test_invalid_image_format_raises():
    with pytest.raises(ValueError):
        _ = from_proto_format(999)  # type: ignore[arg-type]

    with pytest.raises(ValueError):
        _ = from_proto_format_to_n_channels(999)  # type: ignore[arg-type]
