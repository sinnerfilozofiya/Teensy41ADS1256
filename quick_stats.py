#!/usr/bin/env python3
"""
Quick Statistics for ADS1256 Data
Analyze CSV data without external dependencies
"""

import csv
import statistics
import sys
from pathlib import Path

def analyze_csv(filename):
    """Analyze CSV data and show statistics"""
    
    print(f"📊 Analyzing {filename}...")
    
    # Read data
    samples = []
    timestamps = []
    
    try:
        with open(filename, 'r') as f:
            reader = csv.DictReader(f)
            for row in reader:
                try:
                    timestamp = float(row['timestamp'])
                    lc1 = int(row['lc1'])
                    lc2 = int(row['lc2'])
                    lc3 = int(row['lc3'])
                    lc4 = int(row['lc4'])
                    
                    timestamps.append(timestamp)
                    samples.append([lc1, lc2, lc3, lc4])
                    
                except (ValueError, KeyError):
                    continue
        
        if not samples:
            print("❌ No valid data found")
            return
        
        print(f"✅ Loaded {len(samples)} samples")
        
        # Calculate timing statistics
        if len(timestamps) > 1:
            total_time = timestamps[-1] - timestamps[0]
            avg_sps = len(samples) / total_time if total_time > 0 else 0
            data_rate = avg_sps * 12  # 12 bytes per sample
            
            # Calculate sample intervals
            intervals = []
            for i in range(1, len(timestamps)):
                interval = timestamps[i] - timestamps[i-1]
                if 0.0001 < interval < 1.0:  # Filter out unrealistic intervals
                    intervals.append(interval)
            
            if intervals:
                avg_interval = statistics.mean(intervals)
                instantaneous_sps = 1.0 / avg_interval if avg_interval > 0 else 0
                min_interval = min(intervals)
                max_interval = max(intervals)
                max_sps = 1.0 / min_interval if min_interval > 0 else 0
                min_sps = 1.0 / max_interval if max_interval > 0 else 0
            else:
                instantaneous_sps = avg_sps
                min_sps = max_sps = avg_sps
        
        print("\n" + "=" * 80)
        print("📈 PERFORMANCE STATISTICS")
        print("=" * 80)
        print(f"Total Duration:      {total_time:.2f} seconds ({total_time/60:.1f} minutes)")
        print(f"Average Sample Rate: {avg_sps:.1f} SPS")
        print(f"Data Rate:           {data_rate:.0f} bytes/s ({data_rate/1024:.2f} KB/s)")
        print(f"Sample Rate Range:   {min_sps:.1f} - {max_sps:.1f} SPS")
        print(f"Typical Sample Rate: {instantaneous_sps:.1f} SPS")
        
        # Calculate load cell statistics
        print("\n" + "=" * 80)
        print("🏋️  LOAD CELL STATISTICS")
        print("=" * 80)
        
        lc_names = ['LC1', 'LC2', 'LC3', 'LC4']
        
        print(f"{'Channel':<8} {'Current':<12} {'Min':<12} {'Max':<12} {'Mean':<12} {'Std Dev':<10} {'Range':<12}")
        print("-" * 80)
        
        for i in range(4):
            values = [sample[i] for sample in samples]
            
            current = values[-1]
            min_val = min(values)
            max_val = max(values)
            mean_val = statistics.mean(values)
            std_val = statistics.stdev(values) if len(values) > 1 else 0
            range_val = max_val - min_val
            
            print(f"{lc_names[i]:<8} {current:<12} {min_val:<12} {max_val:<12} "
                  f"{mean_val:<12.1f} {std_val:<10.1f} {range_val:<12}")
        
        # Show recent samples
        print(f"\n📋 RECENT SAMPLES (Last 10)")
        print("-" * 80)
        recent_samples = samples[-10:]
        start_idx = len(samples) - len(recent_samples)
        
        for i, sample in enumerate(recent_samples):
            sample_num = start_idx + i + 1
            values_str = " | ".join([f"{v:>8}" for v in sample])
            print(f"#{sample_num:>6}: {values_str}")
        
        # Data quality assessment
        print(f"\n🔍 DATA QUALITY ASSESSMENT")
        print("-" * 80)
        
        for i, name in enumerate(lc_names):
            values = [sample[i] for sample in samples]
            mean_val = statistics.mean(values)
            std_val = statistics.stdev(values) if len(values) > 1 else 0
            
            # Coefficient of variation
            cv = (std_val / abs(mean_val)) * 100 if mean_val != 0 else 0
            
            # Stability check (first 1000 vs last 1000)
            if len(samples) > 2000:
                first_1k = [sample[i] for sample in samples[:1000]]
                last_1k = [sample[i] for sample in samples[-1000:]]
                
                first_mean = statistics.mean(first_1k)
                last_mean = statistics.mean(last_1k)
                drift = last_mean - first_mean
                drift_pct = (drift / abs(first_mean)) * 100 if first_mean != 0 else 0
                
                print(f"{name} - CV: {cv:.2f}% | Drift: {drift:+.1f} ({drift_pct:+.3f}%)")
            else:
                print(f"{name} - CV: {cv:.2f}%")
        
        print(f"\n✅ Analysis complete!")
        
    except Exception as e:
        print(f"❌ Error: {e}")

def main():
    if len(sys.argv) != 2:
        print("Usage: python3 quick_stats.py <csv_file>")
        sys.exit(1)
    
    csv_file = sys.argv[1]
    
    if not Path(csv_file).exists():
        print(f"❌ File not found: {csv_file}")
        sys.exit(1)
    
    analyze_csv(csv_file)

if __name__ == "__main__":
    main()
