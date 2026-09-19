#!/usr/bin/env python3

import argparse
from pathlib import Path

from matplotlib.backends.backend_agg import FigureCanvasAgg
from matplotlib.colors import ListedColormap
from matplotlib.figure import Figure
import numpy as np
import yaml


# Steering Y to differential wheel velocity calibration used by ControlProtocol.
CALIBRATION = np.array(
    [
        [-0.90, -0.282140],
        [-0.80, -0.251145],
        [-0.70, -0.216893],
        [-0.40, -0.119475],
        [-0.20, -0.056844],
        [-0.10, 0.000000],
        [0.00, 0.000000],
        [0.05, 0.027066],
        [0.10, 0.045977],
        [0.20, 0.074416],
        [0.40, 0.142836],
        [0.70, 0.239035],
        [0.80, 0.272361],
    ],
    dtype=float,
)


def default_config_path() -> Path:
    try:
        from ament_index_python.packages import get_package_share_directory

        return (
            Path(get_package_share_directory('ble_hardware_bridge'))
            / 'config'
            / 'ble_hardware_bridge.yaml'
        )
    except (ImportError, LookupError):
        script_path = Path(__file__).resolve()
        source_config = script_path.parents[1] / 'config' / 'ble_hardware_bridge.yaml'
        if source_config.exists():
            return source_config
        return (
            script_path.parents[2]
            / 'share'
            / 'ble_hardware_bridge'
            / 'config'
            / 'ble_hardware_bridge.yaml'
        )


def load_parameters(path: Path) -> dict:
    with path.open('r', encoding='utf-8') as config_file:
        document = yaml.safe_load(config_file)
    try:
        return document['ble_hardware_bridge_node']['ros__parameters']
    except (KeyError, TypeError) as error:
        raise ValueError(f'invalid BLE bridge parameter file: {path}') from error


def differential_from_y(steering_y):
    return np.interp(steering_y, CALIBRATION[:, 0], CALIBRATION[:, 1])


def inverse_differential(target: float) -> float:
    if abs(target) <= 1.0e-12:
        return 0.0
    return float(np.interp(target, CALIBRATION[:, 1], CALIBRATION[:, 0]))


def selected_gear_scale(linear_velocity, parameters):
    reference_maximum = float(parameters['gear_0005_maximum_linear_velocity'])
    gear_0001_scale = (
        float(parameters['gear_0001_maximum_linear_velocity'])
        / reference_maximum
    )
    gear_0003_scale = (
        float(parameters['gear_0003_maximum_linear_velocity'])
        / reference_maximum
    )
    return np.where(
        linear_velocity < float(parameters['gear_0001_selection_threshold']),
        gear_0001_scale,
        np.where(
            linear_velocity < float(parameters['gear_0003_selection_threshold']),
            gear_0003_scale,
            1.0,
        ),
    )


def annotate_intercept(plot, horizontal, vertical, text, color, offset):
    plot.scatter(
        [horizontal],
        [vertical],
        s=34,
        color=color,
        edgecolor='white',
        linewidth=0.7,
        zorder=7,
    )
    plot.annotate(
        text,
        xy=(horizontal, vertical),
        xytext=offset,
        textcoords='offset points',
        fontsize=8,
        color=color,
        fontweight='bold',
        zorder=8,
    )


def annotate_constraint_intersection(
    plot, horizontal, vertical, text, offset, label=None
):
    color = '#5f0f40'
    plot.scatter(
        [horizontal],
        [vertical],
        marker='D',
        s=25,
        color=color,
        edgecolor='white',
        linewidth=0.6,
        zorder=7,
        label=label,
    )
    plot.annotate(
        text,
        xy=(horizontal, vertical),
        xytext=offset,
        textcoords='offset points',
        fontsize=7,
        color=color,
        fontweight='bold',
        zorder=8,
    )


def executable_mask(x_grid, y_grid, parameters):
    wheel_separation = float(parameters['wheel_separation'])
    maximum_wheel_velocity = float(parameters['maximum_wheel_velocity'])
    maximum_angular_velocity = float(parameters['maximum_absolute_angular_velocity'])
    deadzone = float(parameters['joystick_deadzone'])
    maximum_steering_y = float(parameters['maximum_absolute_joystick_y'])
    gain = float(parameters['longitudinal_gain'])
    exponent = float(parameters['longitudinal_exponent'])

    differential = differential_from_y(y_grid)
    linear_velocity = gain * np.power(np.clip(x_grid, 0.0, None), exponent)
    angular_velocity = differential / (0.5 * wheel_separation)
    left_wheel = linear_velocity - differential
    right_wheel = linear_velocity + differential

    calibrated_y = (y_grid >= CALIBRATION[0, 0]) & (y_grid <= CALIBRATION[-1, 0])
    # The inverse mapping returns Y=0 for zero differential, so the flat
    # calibration segment [-0.1, 0) is not emitted by map_twist().
    reachable_y = (y_grid < -0.1) | (y_grid >= 0.0)

    return (
        (x_grid >= deadzone)
        & (x_grid <= 1.0)
        & (np.abs(y_grid) <= maximum_steering_y)
        & calibrated_y
        & reachable_y
        & (np.abs(angular_velocity) <= maximum_angular_velocity)
        & (left_wheel >= 0.0)
        & (left_wheel <= maximum_wheel_velocity)
        & (right_wheel >= 0.0)
        & (right_wheel <= maximum_wheel_velocity)
    )


def plot_joystick_constraints(parameters, output: Path, resolution: int, dpi: int):
    wheel_separation = float(parameters['wheel_separation'])
    maximum_wheel_velocity = float(parameters['maximum_wheel_velocity'])
    maximum_angular_velocity = float(parameters['maximum_absolute_angular_velocity'])
    deadzone = float(parameters['joystick_deadzone'])
    gain = float(parameters['longitudinal_gain'])
    exponent = float(parameters['longitudinal_exponent'])

    axis = np.linspace(-1.0, 1.0, resolution)
    x_grid, y_grid = np.meshgrid(axis, axis)
    mask = executable_mask(x_grid, y_grid, parameters)

    figure = Figure(figsize=(9.2, 8.0), constrained_layout=True)
    FigureCanvasAgg(figure)
    plot = figure.subplots()
    plot.set_facecolor('#eeeeec')
    plot.pcolormesh(
        axis,
        axis,
        mask.astype(int).T,
        shading='nearest',
        cmap=ListedColormap(['#eeeeec', '#4c956c']),
        vmin=0,
        vmax=1,
        rasterized=True,
    )

    negative_y = np.linspace(CALIBRATION[0, 0], -0.100001, 700)
    positive_y = np.linspace(0.0, CALIBRATION[-1, 0], 700)
    boundary_label_used = False
    minimum_label_used = False
    for steering_y in (negative_y, positive_y):
        differential = differential_from_y(steering_y)
        angular_velocity = differential / (0.5 * wheel_separation)
        angular_valid = np.abs(angular_velocity) <= maximum_angular_velocity

        maximum_linear = maximum_wheel_velocity - np.abs(differential)
        maximum_x = np.power(np.clip(maximum_linear / gain, 0.0, None), 1.0 / exponent)
        maximum_x[~angular_valid] = np.nan
        plot.plot(
            steering_y,
            maximum_x,
            color='#7b2cbf',
            linewidth=2.0,
            label='Wheel maximum boundary' if not boundary_label_used else None,
        )
        boundary_label_used = True

        minimum_x = np.power(
            np.clip(np.abs(differential) / gain, 0.0, None), 1.0 / exponent
        )
        minimum_x[~angular_valid] = np.nan
        plot.plot(
            steering_y,
            minimum_x,
            color='#7b2cbf',
            linewidth=1.3,
            linestyle=':',
            label='Wheel nonnegative boundary' if not minimum_label_used else None,
        )
        minimum_label_used = True

    negative_angular_y = inverse_differential(
        -0.5 * wheel_separation * maximum_angular_velocity
    )
    positive_angular_y = inverse_differential(
        0.5 * wheel_separation * maximum_angular_velocity
    )
    plot.axvline(
        negative_angular_y,
        color='#c1121f',
        linewidth=1.6,
        linestyle='-.',
        label='Angular velocity limits',
    )
    plot.axvline(
        positive_angular_y,
        color='#c1121f',
        linewidth=1.6,
        linestyle='-.',
    )
    plot.axhline(
        deadzone,
        color='#e07a1f',
        linewidth=1.8,
        linestyle='--',
        label=f'Forward deadzone X={deadzone:g}',
    )
    plot.axhline(1.0, color='#343a40', linewidth=1.2, label='Joystick X maximum')
    plot.axvspan(
        -0.1,
        0.0,
        facecolor='none',
        edgecolor='#555555',
        hatch='////',
        linewidth=0.0,
        label='Not emitted by inverse calibration',
    )
    plot.scatter(
        [0.0],
        [0.0],
        marker='*',
        s=170,
        color='#1565c0',
        edgecolor='white',
        linewidth=0.8,
        zorder=5,
        label='Executable stop (0, 0)',
    )
    wheel_maximum_x_intercept = np.power(
        maximum_wheel_velocity / gain, 1.0 / exponent
    )
    annotate_intercept(plot, 0.0, deadzone, f'X={deadzone:.3f}', '#e07a1f', (7, 6))
    annotate_intercept(plot, 0.0, 1.0, 'X=1.000', '#343a40', (7, -14))
    annotate_intercept(
        plot,
        0.0,
        wheel_maximum_x_intercept,
        f'X={wheel_maximum_x_intercept:.3f}',
        '#7b2cbf',
        (-75, 7),
    )
    annotate_intercept(
        plot,
        positive_angular_y,
        0.0,
        f'Y=+{positive_angular_y:.3f}',
        '#c1121f',
        (5, 8),
    )
    annotate_intercept(
        plot,
        negative_angular_y,
        0.0,
        f'Y={negative_angular_y:.3f}',
        '#c1121f',
        (-68, 8),
    )

    angular_differential = 0.5 * wheel_separation * maximum_angular_velocity
    wheel_minimum_x_at_angular_limit = np.power(
        angular_differential / gain, 1.0 / exponent
    )
    wheel_maximum_x_at_angular_limit = np.power(
        (maximum_wheel_velocity - angular_differential) / gain,
        1.0 / exponent,
    )
    intersections_at_angular_limits = (
        (wheel_minimum_x_at_angular_limit, -10),
        (deadzone, 5),
        (wheel_maximum_x_at_angular_limit, -10),
        (1.0, -16),
    )
    first_intersection = True
    for angular_y, horizontal_offset in (
        (positive_angular_y, 6),
        (negative_angular_y, -91),
    ):
        for intersection_x, vertical_offset in intersections_at_angular_limits:
            annotate_constraint_intersection(
                plot,
                angular_y,
                intersection_x,
                f'({angular_y:+.3f}, {intersection_x:.3f})',
                (horizontal_offset, vertical_offset),
                'Constraint intersection (Y, X)' if first_intersection else None,
            )
            first_intersection = False

    wheel_margin_at_x_maximum = maximum_wheel_velocity - gain
    positive_wheel_mapping_y = inverse_differential(wheel_margin_at_x_maximum)
    negative_wheel_mapping_y = inverse_differential(-wheel_margin_at_x_maximum)
    annotate_constraint_intersection(
        plot,
        positive_wheel_mapping_y,
        1.0,
        f'({positive_wheel_mapping_y:+.3f}, 1.000)',
        (-112, -31),
    )
    annotate_constraint_intersection(
        plot,
        negative_wheel_mapping_y,
        1.0,
        f'({negative_wheel_mapping_y:+.3f}, 1.000)',
        (7, -31),
    )
    plot.annotate(
        '(0.000, 0.000)',
        xy=(0.0, 0.0),
        xytext=(7, 8),
        textcoords='offset points',
        fontsize=8,
        color='#1565c0',
        fontweight='bold',
        zorder=8,
    )

    # Steering Y is positive to the vehicle's left, so the horizontal axis is
    # reversed. The view is cropped around the executable region; feasibility
    # is still evaluated over the complete [-1, 1] joystick domain.
    plot.set_xlim(0.95, -0.95)
    plot.set_ylim(-0.05, 1.05)
    plot.set_aspect('equal', adjustable='box')
    plot.set_xlabel('Steering joystick Y (positive left)')
    plot.set_ylabel('Forward joystick X (positive up)')
    plot.set_title('BLE executable joystick commands')
    plot.set_xticks(np.linspace(-0.75, 0.75, 7))
    plot.set_yticks(np.linspace(0.0, 1.0, 6))
    plot.grid(color='white', linewidth=0.7, alpha=0.8)
    plot.legend(loc='upper left', bbox_to_anchor=(1.02, 1.0), frameon=False)

    executable_fraction = float(np.count_nonzero(mask)) / float(mask.size)
    plot.text(
        1.02,
        0.02,
        'Green: executable motion\n'
        'Gray: rejected\n'
        f'Grid coverage: {100.0 * executable_fraction:.1f}%',
        transform=plot.transAxes,
        va='bottom',
        fontsize=9,
        color='#343a40',
    )

    output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output, dpi=dpi, facecolor='white')
    figure.clear()


def executable_velocity_mask(linear_grid, angular_grid, parameters):
    wheel_separation = float(parameters['wheel_separation'])
    maximum_wheel_velocity = float(parameters['maximum_wheel_velocity'])
    maximum_angular_velocity = float(parameters['maximum_absolute_angular_velocity'])
    deadzone = float(parameters['joystick_deadzone'])
    maximum_steering_y = float(parameters['maximum_absolute_joystick_y'])
    gain = float(parameters['longitudinal_gain'])
    exponent = float(parameters['longitudinal_exponent'])
    gear_0005_maximum = float(parameters['gear_0005_maximum_linear_velocity'])

    gear_scale = selected_gear_scale(linear_grid, parameters)
    equivalent_linear = linear_grid / gear_scale
    equivalent_angular = angular_grid / gear_scale
    half_separation = 0.5 * wheel_separation
    differential = half_separation * equivalent_angular
    raw_joystick_x = np.power(
        np.clip(equivalent_linear / gain, 0.0, None), 1.0 / exponent
    )
    joystick_x = np.minimum(raw_joystick_x, 1.0)
    joystick_y = np.interp(
        differential, CALIBRATION[:, 1], CALIBRATION[:, 0], left=np.nan, right=np.nan
    )
    joystick_y = np.where(np.abs(differential) <= 1.0e-12, 0.0, joystick_y)
    left_wheel = equivalent_linear - differential
    right_wheel = equivalent_linear + differential

    return (
        (linear_grid >= 0.0)
        & (equivalent_linear <= gear_0005_maximum)
        & (joystick_x >= deadzone)
        & (joystick_x <= 1.0)
        & np.isfinite(joystick_y)
        & (np.abs(joystick_y) <= maximum_steering_y)
        & (np.abs(equivalent_angular) <= maximum_angular_velocity)
        & (left_wheel >= 0.0)
        & (left_wheel <= maximum_wheel_velocity)
        & (right_wheel >= 0.0)
        & (right_wheel <= maximum_wheel_velocity)
    )


def plot_velocity_constraints(parameters, output: Path, resolution: int, dpi: int):
    wheel_separation = float(parameters['wheel_separation'])
    maximum_wheel_velocity = float(parameters['maximum_wheel_velocity'])
    maximum_angular_velocity = float(parameters['maximum_absolute_angular_velocity'])
    deadzone = float(parameters['joystick_deadzone'])
    gain = float(parameters['longitudinal_gain'])
    exponent = float(parameters['longitudinal_exponent'])
    gear_0001_maximum = float(parameters['gear_0001_maximum_linear_velocity'])
    gear_0003_maximum = float(parameters['gear_0003_maximum_linear_velocity'])
    gear_0005_maximum = float(parameters['gear_0005_maximum_linear_velocity'])
    gear_0001_threshold = float(parameters['gear_0001_selection_threshold'])
    gear_0003_threshold = float(parameters['gear_0003_selection_threshold'])
    gear_0003_scale = gear_0003_maximum / gear_0005_maximum
    gear_0001_scale = gear_0001_maximum / gear_0005_maximum

    angular_axis = np.linspace(-1.2, 1.2, resolution)
    linear_axis = np.linspace(-0.1, 1.5, resolution)
    angular_grid, linear_grid = np.meshgrid(angular_axis, linear_axis)
    mask = executable_velocity_mask(linear_grid, angular_grid, parameters)

    figure = Figure(figsize=(10.0, 10.0), constrained_layout=True)
    FigureCanvasAgg(figure)
    grid = figure.add_gridspec(2, 1, height_ratios=(3.2, 1.2))
    plot = figure.add_subplot(grid[0, 0])
    plot.set_facecolor('#eeeeec')
    plot.pcolormesh(
        angular_axis,
        linear_axis,
        mask.astype(int),
        shading='nearest',
        cmap=ListedColormap(['#eeeeec', '#4c956c']),
        vmin=0,
        vmax=1,
        rasterized=True,
    )

    half_separation = 0.5 * wheel_separation
    angular_line = np.linspace(-1.2, 1.2, 800)
    plot.plot(
        angular_line,
        maximum_wheel_velocity + half_separation * angular_line,
        color='#7b2cbf',
        linewidth=2.0,
        label='0005 left wheel maximum reference',
    )
    plot.plot(
        angular_line,
        maximum_wheel_velocity - half_separation * angular_line,
        color='#7b2cbf',
        linewidth=2.0,
        linestyle='--',
        label='0005 right wheel maximum reference',
    )
    plot.plot(
        angular_line,
        half_separation * angular_line,
        color='#7b2cbf',
        linewidth=1.3,
        linestyle=':',
        label='0005 left wheel nonnegative reference',
    )
    plot.plot(
        angular_line,
        -half_separation * angular_line,
        color='#7b2cbf',
        linewidth=1.3,
        linestyle='-.',
        label='0005 right wheel nonnegative reference',
    )

    minimum_linear_velocity = gain * deadzone**exponent
    plot.axhline(
        minimum_linear_velocity,
        color='#e07a1f',
        linewidth=1.8,
        linestyle='--',
        label=f'0005 deadzone reference v={minimum_linear_velocity:.3f} m/s',
    )
    plot.axhline(
        gear_0005_maximum,
        color='#343a40',
        linewidth=1.2,
        label=f'0005 calibrated maximum v={gear_0005_maximum:.3f} m/s',
    )
    plot.axvline(
        -maximum_angular_velocity,
        color='#c1121f',
        linewidth=1.6,
        linestyle='-.',
        label='0005 angular velocity limits',
    )
    plot.axvline(
        maximum_angular_velocity,
        color='#c1121f',
        linewidth=1.6,
        linestyle='-.',
    )
    for threshold, gear_name in (
        (gear_0001_threshold, '0001 -> 0003'),
        (gear_0003_threshold, '0003 -> 0005'),
    ):
        plot.axhline(
            threshold,
            color='#006d77',
            linewidth=1.4,
            linestyle=':',
            label=f'{gear_name} threshold v={threshold:.3f} m/s',
        )
        annotate_intercept(
            plot, 0.0, threshold, f'v={threshold:.3f}', '#006d77', (7, 6)
        )
    plot.scatter(
        [0.0],
        [0.0],
        marker='*',
        s=170,
        color='#1565c0',
        edgecolor='white',
        linewidth=0.8,
        zorder=5,
        label='Executable stop (0, 0)',
    )
    plot.axvline(0.0, color='#777777', linewidth=0.8, zorder=1)
    plot.axhline(0.0, color='#777777', linewidth=0.8, zorder=1)
    annotate_intercept(
        plot,
        0.0,
        minimum_linear_velocity,
        f'v={minimum_linear_velocity:.3f}',
        '#e07a1f',
        (7, 6),
    )
    annotate_intercept(
        plot,
        0.0,
        gear_0005_maximum,
        f'v={gear_0005_maximum:.3f}',
        '#343a40',
        (7, -14),
    )
    annotate_intercept(
        plot,
        0.0,
        maximum_wheel_velocity,
        f'v={maximum_wheel_velocity:.3f}',
        '#7b2cbf',
        (-72, 7),
    )
    annotate_intercept(
        plot,
        maximum_angular_velocity,
        0.0,
        f'omega=+{maximum_angular_velocity:.3f}',
        '#c1121f',
        (5, 8),
    )
    annotate_intercept(
        plot,
        -maximum_angular_velocity,
        0.0,
        f'omega=-{maximum_angular_velocity:.3f}',
        '#c1121f',
        (-82, 8),
    )

    wheel_minimum_at_angular_limit = (
        half_separation * maximum_angular_velocity
    )
    wheel_maximum_at_angular_limit = (
        maximum_wheel_velocity - wheel_minimum_at_angular_limit
    )
    intersections_at_angular_limits = (
        (wheel_minimum_at_angular_limit, -10),
        (minimum_linear_velocity, 5),
        (wheel_maximum_at_angular_limit, -10),
        (gear_0005_maximum, -16),
    )
    first_intersection = True
    for angular_limit, horizontal_offset in (
        (maximum_angular_velocity, 6),
        (-maximum_angular_velocity, -112),
    ):
        for intersection_v, vertical_offset in intersections_at_angular_limits:
            annotate_constraint_intersection(
                plot,
                angular_limit,
                intersection_v,
                f'({angular_limit:+.3f}, {intersection_v:.3f})',
                (horizontal_offset, vertical_offset),
                'Constraint intersection (omega, v)'
                if first_intersection else None,
            )
            first_intersection = False

    plot.annotate(
        '(0.000, 0.000)',
        xy=(0.0, 0.0),
        xytext=(7, 8),
        textcoords='offset points',
        fontsize=8,
        color='#1565c0',
        fontweight='bold',
        zorder=8,
    )

    # Positive angular velocity turns left, so positive values are displayed
    # on the left to match the vehicle-coordinate visualization.
    plot.set_xlim(1.1, -1.1)
    plot.set_ylim(-0.05, 1.43)
    plot.set_ylabel('Linear velocity v [m/s] (positive up)')
    plot.set_title('BLE executable linear and angular velocities')
    plot.set_xticks(np.linspace(-1.0, 1.0, 9))
    plot.set_yticks(np.arange(0.0, 1.41, 0.2))
    plot.tick_params(axis='x', labelbottom=False)
    plot.grid(color='white', linewidth=0.7, alpha=0.8)
    plot.legend(loc='upper left', bbox_to_anchor=(1.02, 1.0), frameon=False)

    executable_fraction = float(np.count_nonzero(mask)) / float(mask.size)
    plot.text(
        1.02,
        0.02,
        'Green: executable motion\n'
        'Gray: rejected\n'
        f'Grid coverage: {100.0 * executable_fraction:.1f}%',
        transform=plot.transAxes,
        va='bottom',
        fontsize=9,
        color='#343a40',
    )
    wheel_axis_intercept = maximum_wheel_velocity / half_separation
    plot.text(
        1.02,
        0.16,
        'Wheel-maximum intersections with v=0\n'
        f'omega=+/-{wheel_axis_intercept:.3f} rad/s (outside view)',
        transform=plot.transAxes,
        va='bottom',
        fontsize=8,
        color='#7b2cbf',
    )
    plot.text(
        1.02,
        0.11,
        'Pair labels use (omega, v); only intersections\n'
        'inside the displayed ranges are marked.',
        transform=plot.transAxes,
        va='bottom',
        fontsize=8,
        color='#5f0f40',
    )

    mapping_plot = figure.add_subplot(grid[1, 0], sharex=plot)
    for gear_name, gear_scale, color in (
        ('0001', gear_0001_scale, '#d1495b'),
        ('0003', gear_0003_scale, '#edae49'),
        ('0005', 1.0, '#0081a7'),
    ):
        negative_angular = np.linspace(
            -gear_scale * maximum_angular_velocity, -1.0e-9, 700
        )
        positive_angular = np.linspace(
            0.0, gear_scale * maximum_angular_velocity, 700
        )
        for angular_values, label in (
            (negative_angular, f'Gear {gear_name}: piecewise Y(omega)'),
            (positive_angular, None),
        ):
            differential = half_separation * angular_values / gear_scale
            steering_y = np.interp(
                differential, CALIBRATION[:, 1], CALIBRATION[:, 0]
            )
            steering_y = np.where(
                np.abs(differential) <= 1.0e-12, 0.0, steering_y
            )
            mapping_plot.plot(
                angular_values,
                steering_y,
                color=color,
                linewidth=2.0,
                label=label,
            )

        breakpoint_angular = (
            gear_scale * CALIBRATION[:, 1] / half_separation
        )
        breakpoint_mask = (
            (np.abs(breakpoint_angular) <= gear_scale * maximum_angular_velocity)
            & (np.abs(CALIBRATION[:, 1]) > 1.0e-12)
        )
        mapping_plot.scatter(
            breakpoint_angular[breakpoint_mask],
            CALIBRATION[breakpoint_mask, 0],
            s=20,
            color=color,
            zorder=4,
        )
    mapping_plot.scatter(
        [0.0],
        [-0.1],
        s=42,
        facecolor='white',
        edgecolor='#1d3557',
        linewidth=1.3,
        zorder=5,
        label='Unused zero-differential endpoint',
    )
    mapping_plot.scatter(
        [0.0],
        [0.0],
        s=42,
        color='#1d3557',
        zorder=5,
        label='Selected value at omega=0',
    )
    mapping_plot.annotate(
        'Y=-0.100',
        xy=(0.0, -0.1),
        xytext=(7, -14),
        textcoords='offset points',
        fontsize=8,
        color='#1d3557',
    )
    mapping_plot.annotate(
        'Y=0.000',
        xy=(0.0, 0.0),
        xytext=(7, 7),
        textcoords='offset points',
        fontsize=8,
        color='#1d3557',
    )
    mapping_plot.plot(
        [0.0, 0.0],
        [-0.1, 0.0],
        color='#555555',
        linewidth=1.0,
        linestyle=':',
    )
    mapping_plot.axhline(0.0, color='#777777', linewidth=0.8)
    mapping_plot.set_ylim(-0.95, 0.85)
    mapping_plot.set_xlabel('Angular velocity omega [rad/s] (positive left)')
    mapping_plot.set_ylabel('Encoded steering Y')
    mapping_plot.set_title(
        'Steering lookup by gear: Y = inverse_calibration((L / 2) omega / scale)'
    )
    mapping_plot.set_xticks(np.linspace(-1.0, 1.0, 9))
    mapping_plot.set_yticks(np.linspace(-0.8, 0.8, 5))
    mapping_plot.grid(color='#dddddd', linewidth=0.7)
    mapping_plot.legend(
        loc='upper left', bbox_to_anchor=(1.02, 1.0), frameon=False, fontsize=8
    )

    output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output, dpi=dpi, facecolor='white')
    figure.clear()


def parse_arguments():
    parser = argparse.ArgumentParser(
        description='Plot joystick X/Y commands accepted by ble_hardware_bridge.'
    )
    parser.add_argument(
        '--config',
        type=Path,
        default=default_config_path(),
        help='BLE bridge YAML parameter file.',
    )
    parser.add_argument(
        '--output',
        type=Path,
        default=Path.cwd() / 'ble_control_constraints.png',
        help='Output PNG path.',
    )
    parser.add_argument(
        '--space',
        choices=('joystick', 'velocity'),
        default='joystick',
        help='Coordinate space to visualize.',
    )
    parser.add_argument('--resolution', type=int, default=1001)
    parser.add_argument('--dpi', type=int, default=160)
    return parser.parse_args()


def main():
    arguments = parse_arguments()
    if arguments.resolution < 101:
        raise ValueError('resolution must be at least 101')
    parameters = load_parameters(arguments.config)
    if arguments.space == 'velocity':
        plot_velocity_constraints(
            parameters, arguments.output, arguments.resolution, arguments.dpi
        )
    else:
        plot_joystick_constraints(
            parameters, arguments.output, arguments.resolution, arguments.dpi
        )
    print(f'wrote {arguments.output.resolve()}')


if __name__ == '__main__':
    main()
