import numpy as np
import pytest

from backend.python.common.util.math import (
    create_transformation_matrix,
    ensure_proper_rotation,
    from_float_list,
    get_translation_rotation_components,
    make_3d_rotation_from_yaw,
    make_transformation_matrix_p_d,
    normalize_vector,
    swap_rotation_components,
)


def test_normalize_vector_unit_length():
    v = np.array([3.0, 4.0, 0.0])
    out = normalize_vector(v)
    assert np.allclose(np.linalg.norm(out), 1.0)
    assert np.allclose(out, np.array([0.6, 0.8, 0.0]))


def test_create_transformation_matrix_shapes_and_components():
    R = np.array([[1.0, 0.0, 0.0], [0.0, 0.0, -1.0], [0.0, 1.0, 0.0]])
    t = np.array([1.0, 2.0, 3.0])
    T = create_transformation_matrix(rotation_matrix=R, translation_vector=t)
    assert T.shape == (4, 4)
    assert np.allclose(T[:3, :3], R)
    assert np.allclose(T[:3, 3], t)
    assert np.allclose(T[3, :], np.array([0.0, 0.0, 0.0, 1.0]))


def test_get_translation_rotation_components_roundtrip():
    R = make_3d_rotation_from_yaw(np.pi / 3)
    t = np.array([10.0, -2.0, 0.5])
    T = create_transformation_matrix(rotation_matrix=R, translation_vector=t)
    t2, R2 = get_translation_rotation_components(T)
    assert np.allclose(t2, t)
    assert np.allclose(R2, R)


def test_make_transformation_matrix_p_d_axes_orthonormal():
    T = make_transformation_matrix_p_d(
        position=np.array([1.0, 2.0, 3.0]),
        direction_vector=np.array([1.0, 0.0, 0.0]),
        z_axis=np.array([0.0, 0.0, 1.0]),
    )
    R = T[:3, :3]
    # Orthonormal columns
    assert np.allclose(R.T @ R, np.eye(3), atol=1e-6)
    # Translation preserved
    assert np.allclose(T[:3, 3], np.array([1.0, 2.0, 3.0]))


def test_ensure_proper_rotation_fixes_reflection():
    # Reflection matrix (det = -1) should be converted to a proper rotation (det = +1)
    R_reflect = np.diag([1.0, 1.0, -1.0])
    R_fixed = ensure_proper_rotation(R_reflect)
    assert pytest.approx(np.linalg.det(R_fixed), abs=1e-6) == 1.0
    assert np.allclose(R_fixed.T @ R_fixed, np.eye(3), atol=1e-6)


def test_from_float_list_happy_path():
    mat = from_float_list([1.0, 2.0, 3.0, 4.0], rows=2, cols=2)
    assert np.allclose(mat, np.array([[1.0, 2.0], [3.0, 4.0]]))


def test_from_float_list_raises_on_wrong_size():
    with pytest.raises(ValueError):
        _ = from_float_list([1.0, 2.0, 3.0], rows=2, cols=2)


def test_make_3d_rotation_from_yaw_basic_angles():
    R0 = make_3d_rotation_from_yaw(0.0)
    assert np.allclose(R0, np.eye(3))

    R90 = make_3d_rotation_from_yaw(np.pi / 2)
    assert np.allclose(
        R90, np.array([[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])
    )


def test_swap_rotation_components_only_rotation_swapped():
    T1 = np.eye(4)
    T2 = np.eye(4)
    T1[:3, :3] = make_3d_rotation_from_yaw(0.0)
    T2[:3, :3] = make_3d_rotation_from_yaw(np.pi / 4)
    T1[:3, 3] = np.array([1.0, 2.0, 3.0])
    T2[:3, 3] = np.array([-1.0, -2.0, -3.0])

    A, B = swap_rotation_components(T_one=T1, T_two=T2, R_side_size=3)
    assert np.allclose(A[:3, :3], T2[:3, :3])
    assert np.allclose(B[:3, :3], T1[:3, :3])
    # Translations unchanged
    assert np.allclose(A[:3, 3], T1[:3, 3])
    assert np.allclose(B[:3, 3], T2[:3, 3])
