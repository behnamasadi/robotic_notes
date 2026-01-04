#!/usr/bin/env python3
"""
Analyze IMU Sampling Rate from Data

This script computes the actual sampling frequency from IMU timestamp data
by analyzing time differences between consecutive samples.
"""

import numpy as np
import pandas as pd
from pathlib import Path
import argparse


def analyze_sampling_rate(csv_path: Path, verbose: bool = True):
    """
    Analyze IMU sampling rate from timestamp data.
    
    Args:
        csv_path: Path to IMU data CSV file
        verbose: Whether to print detailed statistics
        
    Returns:
        dict: Dictionary containing sampling rate statistics
    """
    # Read CSV file
    df = pd.read_csv(csv_path, comment='#')
    
    # Get timestamp column (first column)
    timestamps_ns = df.iloc[:, 0].values
    
    # Convert to seconds for easier analysis
    timestamps_s = timestamps_ns * 1e-9
    
    # Compute time differences between consecutive samples
    dt = np.diff(timestamps_s)
    
    # Remove any zero or negative differences (shouldn't happen, but safety check)
    dt = dt[dt > 0]
    
    # Calculate statistics
    mean_dt = np.mean(dt)
    median_dt = np.median(dt)
    std_dt = np.std(dt)
    min_dt = np.min(dt)
    max_dt = np.max(dt)
    
    # Calculate frequencies
    mean_freq = 1.0 / mean_dt
    median_freq = 1.0 / median_dt
    
    # Calculate jitter (coefficient of variation)
    jitter_cv = std_dt / mean_dt  # Coefficient of variation
    jitter_percent = jitter_cv * 100
    
    # Total duration and number of samples
    total_duration = timestamps_s[-1] - timestamps_s[0]
    num_samples = len(timestamps_s)
    expected_samples = total_duration * mean_freq
    
    results = {
        'mean_dt': mean_dt,
        'median_dt': median_dt,
        'std_dt': std_dt,
        'min_dt': min_dt,
        'max_dt': max_dt,
        'mean_freq': mean_freq,
        'median_freq': median_freq,
        'jitter_cv': jitter_cv,
        'jitter_percent': jitter_percent,
        'total_duration': total_duration,
        'num_samples': num_samples,
        'expected_samples': expected_samples,
    }
    
    if verbose:
        print("=" * 60)
        print("IMU Sampling Rate Analysis")
        print("=" * 60)
        print(f"\nData file: {csv_path}")
        print(f"Total samples: {num_samples:,}")
        print(f"Total duration: {total_duration:.3f} seconds ({total_duration/60:.2f} minutes)")
        print(f"\nTime Difference Statistics:")
        print(f"  Mean dt:     {mean_dt*1e3:.6f} ms")
        print(f"  Median dt:   {median_dt*1e3:.6f} ms")
        print(f"  Std dt:      {std_dt*1e6:.3f} μs")
        print(f"  Min dt:      {min_dt*1e3:.6f} ms")
        print(f"  Max dt:      {max_dt*1e3:.6f} ms")
        print(f"\nSampling Frequency:")
        print(f"  Mean frequency:   {mean_freq:.6f} Hz")
        print(f"  Median frequency: {median_freq:.6f} Hz")
        print(f"\nJitter Analysis:")
        print(f"  Coefficient of variation: {jitter_cv:.6f}")
        print(f"  Jitter: {jitter_percent:.4f}%")
        
        if jitter_percent < 0.1:
            print(f"  Status: Very stable sampling (excellent)")
        elif jitter_percent < 1.0:
            print(f"  Status: Stable sampling (good)")
        elif jitter_percent < 5.0:
            print(f"  Status: Moderate jitter (acceptable)")
        else:
            print(f"  Status: High jitter (may need attention)")
        
        print(f"\nExpected vs Actual Samples:")
        print(f"  Expected: {expected_samples:.1f}")
        print(f"  Actual:   {num_samples}")
        print(f"  Difference: {abs(num_samples - expected_samples):.1f} samples")
        print("=" * 60)
    
    return results


def main():
    parser = argparse.ArgumentParser(
        description='Analyze IMU sampling rate from timestamp data')
    parser.add_argument(
        '--data-file',
        type=str,
        default='../data/machine_hall/MH_01_easy/MH_01_easy/mav0/imu0/data.csv',
        help='Path to IMU data CSV file'
    )
    parser.add_argument(
        '--quiet',
        action='store_true',
        help='Suppress verbose output'
    )
    
    args = parser.parse_args()
    
    # Resolve path
    script_dir = Path(__file__).resolve().parent.parent
    data_file = Path(args.data_file)
    if not data_file.is_absolute():
        data_file = script_dir / data_file
    
    if not data_file.exists():
        print(f"Error: Data file not found at {data_file}")
        return 1
    
    results = analyze_sampling_rate(data_file, verbose=not args.quiet)
    
    return 0


if __name__ == '__main__':
    exit(main())
