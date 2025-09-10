#!/usr/bin/env python3
"""
ADS1256 Data Analyzer
Analyze collected CSV data and show comprehensive statistics
"""

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import argparse
import sys
from pathlib import Path

class DataAnalyzer:
    def __init__(self, csv_file):
        self.csv_file = csv_file
        self.data = None
        self.load_data()
    
    def load_data(self):
        """Load CSV data"""
        try:
            self.data = pd.read_csv(self.csv_file)
            print(f"✓ Loaded {len(self.data)} samples from {self.csv_file}")
            
            # Calculate time differences for rate analysis
            if 'timestamp' in self.data.columns:
                self.data['time_diff'] = self.data['timestamp'].diff()
                self.data['sample_rate'] = 1.0 / self.data['time_diff']
            
        except Exception as e:
            print(f"✗ Error loading data: {e}")
            sys.exit(1)
    
    def calculate_overall_stats(self):
        """Calculate overall statistics"""
        if self.data is None or len(self.data) == 0:
            return None
        
        # Time analysis
        if 'timestamp' in self.data.columns:
            total_time = self.data['timestamp'].iloc[-1] - self.data['timestamp'].iloc[0]
            avg_sample_rate = len(self.data) / total_time if total_time > 0 else 0
            
            # Data rate calculation (12 bytes per sample)
            avg_data_rate = avg_sample_rate * 12
            
            # Sample rate statistics
            sample_rates = self.data['sample_rate'].dropna()
            
            time_stats = {
                'total_time': total_time,
                'avg_sample_rate': avg_sample_rate,
                'avg_data_rate': avg_data_rate,
                'min_sample_rate': sample_rates.min() if len(sample_rates) > 0 else 0,
                'max_sample_rate': sample_rates.max() if len(sample_rates) > 0 else 0,
                'std_sample_rate': sample_rates.std() if len(sample_rates) > 0 else 0
            }
        else:
            time_stats = {}
        
        # Load cell statistics
        lc_columns = ['lc1', 'lc2', 'lc3', 'lc4']
        lc_stats = {}
        
        for col in lc_columns:
            if col in self.data.columns:
                values = self.data[col]
                lc_stats[col] = {
                    'count': len(values),
                    'mean': values.mean(),
                    'std': values.std(),
                    'min': values.min(),
                    'max': values.max(),
                    'range': values.max() - values.min(),
                    'median': values.median(),
                    'q25': values.quantile(0.25),
                    'q75': values.quantile(0.75)
                }
        
        return {
            'time_stats': time_stats,
            'lc_stats': lc_stats,
            'total_samples': len(self.data)
        }
    
    def print_comprehensive_report(self):
        """Print detailed analysis report"""
        stats = self.calculate_overall_stats()
        if not stats:
            print("No data to analyze")
            return
        
        print("=" * 100)
        print("📊 ADS1256 DATA ANALYSIS REPORT")
        print("=" * 100)
        
        # Dataset overview
        print(f"📁 Dataset: {Path(self.csv_file).name}")
        print(f"📈 Total Samples: {stats['total_samples']:,}")
        
        # Time analysis
        if stats['time_stats']:
            ts = stats['time_stats']
            print(f"\n⏱️  TIMING ANALYSIS")
            print(f"   Total Duration:    {ts['total_time']:.2f} seconds ({ts['total_time']/60:.1f} minutes)")
            print(f"   Average SPS:       {ts['avg_sample_rate']:.1f} samples/second")
            print(f"   Average Data Rate: {ts['avg_data_rate']:.0f} bytes/s ({ts['avg_data_rate']/1024:.2f} KB/s)")
            print(f"   SPS Range:         {ts['min_sample_rate']:.1f} - {ts['max_sample_rate']:.1f}")
            print(f"   SPS Std Dev:       {ts['std_sample_rate']:.1f}")
        
        # Load cell analysis
        print(f"\n🏋️  LOAD CELL ANALYSIS")
        print("-" * 100)
        print(f"{'Channel':<8} {'Mean':<12} {'Std Dev':<10} {'Min':<12} {'Max':<12} {'Range':<12} {'Median':<12}")
        print("-" * 100)
        
        for lc, stats_data in stats['lc_stats'].items():
            print(f"{lc.upper():<8} {stats_data['mean']:<12.1f} {stats_data['std']:<10.1f} "
                  f"{stats_data['min']:<12} {stats_data['max']:<12} {stats_data['range']:<12} "
                  f"{stats_data['median']:<12.1f}")
        
        # Data quality analysis
        print(f"\n🔍 DATA QUALITY")
        for lc, stats_data in stats['lc_stats'].items():
            # Calculate coefficient of variation (CV)
            cv = (stats_data['std'] / abs(stats_data['mean'])) * 100 if stats_data['mean'] != 0 else 0
            print(f"   {lc.upper()} Coefficient of Variation: {cv:.2f}%")
        
        # Stability analysis (last 1000 samples vs first 1000 samples)
        if len(self.data) > 2000:
            print(f"\n📊 STABILITY ANALYSIS (First 1000 vs Last 1000 samples)")
            first_1k = self.data.head(1000)
            last_1k = self.data.tail(1000)
            
            for col in ['lc1', 'lc2', 'lc3', 'lc4']:
                if col in self.data.columns:
                    first_mean = first_1k[col].mean()
                    last_mean = last_1k[col].mean()
                    drift = last_mean - first_mean
                    drift_pct = (drift / abs(first_mean)) * 100 if first_mean != 0 else 0
                    
                    print(f"   {col.upper()} Drift: {drift:+.1f} ({drift_pct:+.2f}%)")
    
    def plot_data(self, save_plots=False):
        """Create visualization plots"""
        if self.data is None:
            return
        
        # Create subplots
        fig, axes = plt.subplots(2, 2, figsize=(15, 10))
        fig.suptitle('ADS1256 Load Cell Data Analysis', fontsize=16)
        
        # Plot 1: Time series of all channels
        ax1 = axes[0, 0]
        for col in ['lc1', 'lc2', 'lc3', 'lc4']:
            if col in self.data.columns:
                ax1.plot(self.data.index, self.data[col], label=col.upper(), alpha=0.7)
        ax1.set_title('Load Cell Values Over Time')
        ax1.set_xlabel('Sample Number')
        ax1.set_ylabel('ADC Value')
        ax1.legend()
        ax1.grid(True, alpha=0.3)
        
        # Plot 2: Sample rate over time
        ax2 = axes[0, 1]
        if 'sample_rate' in self.data.columns:
            sample_rates = self.data['sample_rate'].dropna()
            ax2.plot(sample_rates.index, sample_rates.values, alpha=0.7)
            ax2.set_title('Sample Rate Over Time')
            ax2.set_xlabel('Sample Number')
            ax2.set_ylabel('Samples/Second')
            ax2.grid(True, alpha=0.3)
        
        # Plot 3: Histograms
        ax3 = axes[1, 0]
        for i, col in enumerate(['lc1', 'lc2', 'lc3', 'lc4']):
            if col in self.data.columns:
                ax3.hist(self.data[col], bins=50, alpha=0.6, label=col.upper())
        ax3.set_title('Value Distribution')
        ax3.set_xlabel('ADC Value')
        ax3.set_ylabel('Frequency')
        ax3.legend()
        ax3.grid(True, alpha=0.3)
        
        # Plot 4: Recent data (last 1000 samples)
        ax4 = axes[1, 1]
        recent_data = self.data.tail(1000)
        for col in ['lc1', 'lc2', 'lc3', 'lc4']:
            if col in recent_data.columns:
                ax4.plot(recent_data.index, recent_data[col], label=col.upper(), alpha=0.7)
        ax4.set_title('Recent Data (Last 1000 samples)')
        ax4.set_xlabel('Sample Number')
        ax4.set_ylabel('ADC Value')
        ax4.legend()
        ax4.grid(True, alpha=0.3)
        
        plt.tight_layout()
        
        if save_plots:
            plot_file = self.csv_file.replace('.csv', '_analysis.png')
            plt.savefig(plot_file, dpi=300, bbox_inches='tight')
            print(f"✓ Plots saved to {plot_file}")
        
        plt.show()
    
    def export_summary(self, output_file=None):
        """Export summary statistics to file"""
        if output_file is None:
            output_file = self.csv_file.replace('.csv', '_summary.txt')
        
        stats = self.calculate_overall_stats()
        
        with open(output_file, 'w') as f:
            f.write("ADS1256 Data Analysis Summary\n")
            f.write("=" * 50 + "\n\n")
            
            f.write(f"Dataset: {Path(self.csv_file).name}\n")
            f.write(f"Total Samples: {stats['total_samples']:,}\n\n")
            
            if stats['time_stats']:
                ts = stats['time_stats']
                f.write("Timing Statistics:\n")
                f.write(f"  Duration: {ts['total_time']:.2f} seconds\n")
                f.write(f"  Average SPS: {ts['avg_sample_rate']:.1f}\n")
                f.write(f"  Data Rate: {ts['avg_data_rate']/1024:.2f} KB/s\n\n")
            
            f.write("Load Cell Statistics:\n")
            for lc, lc_stats in stats['lc_stats'].items():
                f.write(f"  {lc.upper()}:\n")
                f.write(f"    Mean: {lc_stats['mean']:.1f}\n")
                f.write(f"    Std Dev: {lc_stats['std']:.1f}\n")
                f.write(f"    Range: {lc_stats['min']} to {lc_stats['max']}\n\n")
        
        print(f"✓ Summary exported to {output_file}")

def main():
    parser = argparse.ArgumentParser(description='Analyze ADS1256 CSV data')
    parser.add_argument('csv_file', help='CSV file to analyze')
    parser.add_argument('--plot', '-p', action='store_true', help='Show plots')
    parser.add_argument('--save-plots', '-s', action='store_true', help='Save plots to file')
    parser.add_argument('--export', '-e', action='store_true', help='Export summary to file')
    
    args = parser.parse_args()
    
    if not Path(args.csv_file).exists():
        print(f"✗ File not found: {args.csv_file}")
        sys.exit(1)
    
    analyzer = DataAnalyzer(args.csv_file)
    
    # Print comprehensive report
    analyzer.print_comprehensive_report()
    
    # Show plots if requested
    if args.plot or args.save_plots:
        try:
            analyzer.plot_data(save_plots=args.save_plots)
        except ImportError:
            print("⚠️  Matplotlib not available for plotting")
    
    # Export summary if requested
    if args.export:
        analyzer.export_summary()

if __name__ == "__main__":
    main()
