#!/usr/bin/env python3

import argparse
import csv
import json
import math
import os
import sys
from io import StringIO


DEFAULT_POINTS_DIR = os.path.join(
    os.path.expanduser('~'),
    'lidar',
    'lio',
    'maps',
    'points',
)
DEFAULT_CHECKS_FILE = os.path.join(DEFAULT_POINTS_DIR, 'point_checks.json')


def parse_points(value):
    if value is None:
        return None

    point_ids = []
    for part in value.replace(',', ' ').split():
        if '-' in part:
            start, end = part.split('-', 1)
            start = int(start)
            end = int(end)
            step = 1 if start <= end else -1
            point_ids.extend(range(start, end + step, step))
        else:
            point_ids.append(int(part))

    return set(point_ids)


def load_results(path):
    with open(path, 'r', encoding='utf-8') as file:
        data = json.load(file)

    results = data.get('checks', data.get('results'))
    if not isinstance(results, list):
        raise ValueError('Invalid checks file: missing "checks" list.')

    return data, sorted(results, key=lambda item: item.get('point_index', 0))


def point_id(result):
    return int(result.get('point_id', result.get('point_index', 0) + 1))


def filter_results(results, point_ids):
    if point_ids is None:
        return results
    return [result for result in results if point_id(result) in point_ids]


def scale_error(value, unit):
    if unit == 'mm':
        return value * 1000.0
    return value


def error_unit_label(unit):
    return 'mm' if unit == 'mm' else 'm'


def format_float(value, digits):
    return f'{value:.{digits}f}'


def render_ascii_table(headers, rows, aligns=None):
    if aligns is None:
        aligns = ['right'] * len(headers)

    widths = [len(header) for header in headers]
    for row in rows:
        for index, cell in enumerate(row):
            widths[index] = max(widths[index], len(str(cell)))

    separator = '+-' + '-+-'.join('-' * width for width in widths) + '-+'

    def render_row(row):
        cells = []
        for index, cell in enumerate(row):
            text = str(cell)
            if aligns[index] == 'left':
                cells.append(text.ljust(widths[index]))
            else:
                cells.append(text.rjust(widths[index]))
        return '| ' + ' | '.join(cells) + ' |'

    lines = [
        separator,
        render_row(headers),
        separator,
    ]
    lines.extend(render_row(row) for row in rows)
    lines.append(separator)
    return '\n'.join(lines)


def build_rows(results, unit):
    rows = []
    for result in results:
        reference = result['reference']['position']
        measured = result['measured']['position']
        error = result['error']
        rows.append({
            'point': point_id(result),
            'ref_x_m': reference['x'],
            'ref_y_m': reference['y'],
            'ref_z_m': reference['z'],
            'measured_x_m': measured['x'],
            'measured_y_m': measured['y'],
            'measured_z_m': measured['z'],
            f'dx_{unit}': scale_error(error['dx'], unit),
            f'dy_{unit}': scale_error(error['dy'], unit),
            f'dz_{unit}': scale_error(error['dz'], unit),
            f'horizontal_error_{unit}': scale_error(error['horizontal_error'], unit),
            f'error_3d_{unit}': scale_error(error['error_3d'], unit),
            'yaw_error_deg': error.get('yaw_error_deg'),
        })
    return rows


def summarize(results, unit):
    if not results:
        return {}

    horizontal = [
        scale_error(item['error']['horizontal_error'], unit)
        for item in results
    ]
    error_3d = [
        scale_error(item['error']['error_3d'], unit)
        for item in results
    ]
    yaw = [
        abs(item['error']['yaw_error_deg'])
        for item in results
        if 'yaw_error_deg' in item['error']
    ]

    summary = {
        'horizontal_avg': sum(horizontal) / len(horizontal),
        'horizontal_max': max(horizontal),
        'horizontal_min': min(horizontal),
        'horizontal_rmse': math.sqrt(
            sum(value * value for value in horizontal) / len(horizontal)
        ),
        'error_3d_avg': sum(error_3d) / len(error_3d),
        'error_3d_max': max(error_3d),
        'error_3d_min': min(error_3d),
        'error_3d_rmse': math.sqrt(
            sum(value * value for value in error_3d) / len(error_3d)
        ),
    }

    if yaw:
        summary.update({
            'yaw_abs_avg_deg': sum(yaw) / len(yaw),
            'yaw_abs_max_deg': max(yaw),
            'yaw_abs_min_deg': min(yaw),
        })

    return summary


def render_terminal(data, results, unit):
    rows = build_rows(results, unit)
    summary = summarize(results, unit)
    unit_label = error_unit_label(unit)
    error_digits = 1 if unit == 'mm' else 4

    lines = [
        'Point check report',
        f'  checks: {data.get("_checks_file", "")}',
        f'  points : {data.get("points_file", "")}',
        f'  updated: {data.get("updated_at", "")}',
        f'  count  : {len(results)}',
        '',
        f'Error by point ({unit_label})',
    ]

    error_headers = [
        'Pt',
        f'dx {unit_label}',
        f'dy {unit_label}',
        f'dz {unit_label}',
        f'XY {unit_label}',
        f'3D {unit_label}',
        'Yaw deg',
    ]
    error_rows = []
    for row in rows:
        yaw = row['yaw_error_deg']
        yaw_text = '' if yaw is None else format_float(yaw, 3)
        error_rows.append([
            row['point'],
            format_float(row[f'dx_{unit}'], error_digits),
            format_float(row[f'dy_{unit}'], error_digits),
            format_float(row[f'dz_{unit}'], error_digits),
            format_float(row[f'horizontal_error_{unit}'], error_digits),
            format_float(row[f'error_3d_{unit}'], error_digits),
            yaw_text,
        ])
    lines.append(render_ascii_table(error_headers, error_rows))

    if summary:
        lines.extend([
            '',
            f'Summary ({unit_label})',
        ])
        summary_headers = ['Metric', 'Avg', 'Max', 'Min', 'RMSE']
        summary_rows = [
            [
                'XY',
                format_float(summary['horizontal_avg'], error_digits),
                format_float(summary['horizontal_max'], error_digits),
                format_float(summary['horizontal_min'], error_digits),
                format_float(summary['horizontal_rmse'], error_digits),
            ],
            [
                '3D',
                format_float(summary['error_3d_avg'], error_digits),
                format_float(summary['error_3d_max'], error_digits),
                format_float(summary['error_3d_min'], error_digits),
                format_float(summary['error_3d_rmse'], error_digits),
            ],
        ]
        lines.append(
            render_ascii_table(
                summary_headers,
                summary_rows,
                aligns=['left', 'right', 'right', 'right', 'right'],
            )
        )

        if 'yaw_abs_avg_deg' in summary:
            lines.extend([
                '',
                'Yaw absolute error (deg)',
                render_ascii_table(
                    ['Avg', 'Max', 'Min'],
                    [[
                        format_float(summary['yaw_abs_avg_deg'], 3),
                        format_float(summary['yaw_abs_max_deg'], 3),
                        format_float(summary['yaw_abs_min_deg'], 3),
                    ]],
                ),
            ])

    lines.extend([
        '',
        'Position detail (m)',
    ])
    position_headers = ['Pt', 'Ref x', 'Ref y', 'Ref z', 'Now x', 'Now y', 'Now z']
    position_rows = []
    for row in rows:
        position_rows.append([
            row['point'],
            format_float(row['ref_x_m'], 4),
            format_float(row['ref_y_m'], 4),
            format_float(row['ref_z_m'], 4),
            format_float(row['measured_x_m'], 4),
            format_float(row['measured_y_m'], 4),
            format_float(row['measured_z_m'], 4),
        ])
    lines.append(render_ascii_table(position_headers, position_rows))

    return '\n'.join(lines) + '\n'


def render_markdown(data, results, unit):
    rows = build_rows(results, unit)
    unit_label = error_unit_label(unit)
    error_digits = 1 if unit == 'mm' else 4

    lines = [
        f'Checks file: `{data.get("_checks_file", "")}`',
        f'Points file: `{data.get("points_file", "")}`',
        f'Updated at: `{data.get("updated_at", "")}`',
        '',
        (
            f'| Point | Ref x (m) | Ref y (m) | Ref z (m) | '
            f'Measured x (m) | Measured y (m) | Measured z (m) | '
            f'dx ({unit_label}) | dy ({unit_label}) | dz ({unit_label}) | '
            f'Horizontal ({unit_label}) | 3D ({unit_label}) | Yaw (deg) |'
        ),
        '|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|',
    ]

    for row in rows:
        yaw = row['yaw_error_deg']
        yaw_text = '' if yaw is None else format_float(yaw, 3)
        lines.append(
            '| {point} | {ref_x_m:.4f} | {ref_y_m:.4f} | {ref_z_m:.4f} | '
            '{measured_x_m:.4f} | {measured_y_m:.4f} | {measured_z_m:.4f} | '
            f'{format_float(row[f"dx_{unit}"], error_digits)} | '
            f'{format_float(row[f"dy_{unit}"], error_digits)} | '
            f'{format_float(row[f"dz_{unit}"], error_digits)} | '
            f'{format_float(row[f"horizontal_error_{unit}"], error_digits)} | '
            f'{format_float(row[f"error_3d_{unit}"], error_digits)} | '
            f'{yaw_text} |'
            .format(**row)
        )

    summary = summarize(results, unit)
    if summary:
        lines.extend([
            '',
            f'| Metric | Avg ({unit_label}) | Max ({unit_label}) | '
            f'Min ({unit_label}) | RMSE ({unit_label}) |',
            '|---|---:|---:|---:|---:|',
            (
                '| Horizontal | '
                f'{format_float(summary["horizontal_avg"], error_digits)} | '
                f'{format_float(summary["horizontal_max"], error_digits)} | '
                f'{format_float(summary["horizontal_min"], error_digits)} | '
                f'{format_float(summary["horizontal_rmse"], error_digits)} |'
            ),
            (
                '| 3D | '
                f'{format_float(summary["error_3d_avg"], error_digits)} | '
                f'{format_float(summary["error_3d_max"], error_digits)} | '
                f'{format_float(summary["error_3d_min"], error_digits)} | '
                f'{format_float(summary["error_3d_rmse"], error_digits)} |'
            ),
        ])

        if 'yaw_abs_avg_deg' in summary:
            lines.extend([
                '',
                (
                    'Yaw abs error: '
                    f'avg={summary["yaw_abs_avg_deg"]:.3f} deg, '
                    f'max={summary["yaw_abs_max_deg"]:.3f} deg, '
                    f'min={summary["yaw_abs_min_deg"]:.3f} deg'
                ),
            ])

    return '\n'.join(lines) + '\n'


def render_csv(results, unit):
    rows = build_rows(results, unit)
    output = StringIO()
    fieldnames = list(rows[0].keys()) if rows else ['point']
    writer = csv.DictWriter(output, fieldnames=fieldnames)
    writer.writeheader()
    writer.writerows(rows)
    return output.getvalue()


def build_parser():
    parser = argparse.ArgumentParser(
        description='Create a table from point check JSON.',
    )
    parser.add_argument(
        '--checks-file',
        '--results-file',
        dest='checks_file',
        default=DEFAULT_CHECKS_FILE,
        help=f'Point check JSON path. Default: {DEFAULT_CHECKS_FILE}',
    )
    parser.add_argument(
        '--points',
        default=None,
        help='Point ids to include, for example: 2,3,4 or 1-4.',
    )
    parser.add_argument(
        '--unit',
        choices=['mm', 'm'],
        default='mm',
        help='Error unit for dx/dy/dz and summary. Default: mm.',
    )
    parser.add_argument(
        '--format',
        choices=['terminal', 'markdown', 'csv'],
        default='terminal',
        help='Output format. Default: terminal.',
    )
    parser.add_argument(
        '--output',
        default=None,
        help='Optional output file path. Prints to terminal when omitted.',
    )
    return parser


def main(argv=None):
    parser = build_parser()
    args = parser.parse_args(argv)

    checks_file = os.path.expanduser(args.checks_file)
    point_ids = parse_points(args.points)

    try:
        data, results = load_results(checks_file)
        data['_checks_file'] = checks_file
        results = filter_results(results, point_ids)
    except (OSError, ValueError, json.JSONDecodeError) as exc:
        print(f'Failed to read point check JSON: {exc}', file=sys.stderr)
        return 1

    if not results:
        print('No matching results found.', file=sys.stderr)
        return 1

    if args.format == 'csv':
        report = render_csv(results, args.unit)
    elif args.format == 'markdown':
        report = render_markdown(data, results, args.unit)
    else:
        report = render_terminal(data, results, args.unit)

    if args.output:
        output_path = os.path.expanduser(args.output)
        os.makedirs(os.path.dirname(os.path.abspath(output_path)), exist_ok=True)
        with open(output_path, 'w', encoding='utf-8') as file:
            file.write(report)
    else:
        print(report, end='')

    return 0


if __name__ == '__main__':
    raise SystemExit(main())
