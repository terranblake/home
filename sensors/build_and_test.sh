#!/bin/bash
#############################################################################
# ESP-NOW Temperature Sensor - Build and Test Script
# 
# This script builds and optionally flashes the ESP-NOW sensor components
# Usage: ./build_and_test.sh [build|flash|monitor]
#############################################################################

set -e  # Exit on any error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Function to print colored output
print_status() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Build all working environments
build_all() {
    print_status "Building all ESP-NOW sensor components..."
    
    # Build Recovery Transmitter (for testing)
    print_status "Building Recovery Transmitter (debug/testing)..."
    if pio run -e recovery_transmitter; then
        print_success "Recovery Transmitter built successfully"
    else
        print_error "Recovery Transmitter build failed"
        exit 1
    fi
    
    # Build ESP-NOW Receiver (gateway)
    print_status "Building ESP-NOW Receiver (gateway)..."
    if pio run -e espnow_receiver; then
        print_success "ESP-NOW Receiver built successfully"
    else
        print_error "ESP-NOW Receiver build failed"
        exit 1
    fi
    
    # Build Battery Transmitter (production)
    print_status "Building Battery Transmitter (production)..."
    if pio run -e battery_transmitter; then
        print_success "Battery Transmitter built successfully"
    else
        print_error "Battery Transmitter build failed"
        exit 1
    fi
    
    print_success "All components built successfully!"
    print_warning "Note: ULP Receiver requires advanced ESP-IDF setup (not built)"
}

# Flash specific environment
flash_env() {
    local env=$1
    print_status "Flashing $env..."
    
    if pio run -e $env -t upload; then
        print_success "$env flashed successfully"
    else
        print_error "$env flash failed"
        exit 1
    fi
}

# Monitor serial output
monitor_env() {
    local env=$1
    print_status "Starting serial monitor for $env..."
    print_warning "Press Ctrl+C to exit monitor"
    pio device monitor -e $env
}

# Main script logic
case "${1:-build}" in
    "build")
        build_all
        ;;
    "flash")
        if [ -z "$2" ]; then
            print_error "Usage: $0 flash <environment>"
            print_status "Available environments:"
            echo "  - recovery_transmitter (testing)"
            echo "  - espnow_receiver (gateway)"  
            echo "  - battery_transmitter (production)"
            exit 1
        fi
        flash_env $2
        ;;
    "monitor")
        if [ -z "$2" ]; then
            print_error "Usage: $0 monitor <environment>"
            exit 1
        fi
        monitor_env $2
        ;;
    "test")
        print_status "Running complete test sequence..."
        build_all
        
        print_status "Test sequence complete!"
        print_warning "For hardware testing:"
        echo "  1. Flash receiver: ./build_and_test.sh flash espnow_receiver"
        echo "  2. Flash sensor: ./build_and_test.sh flash recovery_transmitter"  
        echo "  3. Monitor output: ./build_and_test.sh monitor recovery_transmitter"
        ;;
    *)
        print_status "ESP-NOW Temperature Sensor Build Script"
        echo ""
        echo "Usage: $0 [build|flash|monitor|test]"
        echo ""
        echo "Commands:"
        echo "  build   - Build all components (default)"
        echo "  flash   - Flash specific environment to device"
        echo "  monitor - Start serial monitor for environment"
        echo "  test    - Run complete build test"
        echo ""
        echo "Examples:"
        echo "  $0 build                              # Build all"
        echo "  $0 flash recovery_transmitter         # Flash test sensor"
        echo "  $0 flash espnow_receiver             # Flash gateway"
        echo "  $0 monitor recovery_transmitter       # Monitor sensor output"
        ;;
esac
