# SDR
# 🛰️ Software Defined Radar (SDR) – MATLAB Simulink Project

This project focuses on the **design and simulation of a Software Defined Radar (SDR)** system using **MATLAB Simulink**, complete with a **Graphical User Interface (GUI)** for real-time control and visualization. Developed as part of a 10-week internship at **Military College of Electronics and Mechanical Engineering (MCEME), Secunderabad**, this work showcases the potential of reconfigurable radar systems in modern applications.

## 📌 Project Highlights

- **Radar Simulation** using Simulink including:
  - Transmitter & Receiver blocks
  - Free-space channel modeling
  - Target object and radar platform setup
  - Doppler and Range processing
- **GUI Implementation** in MATLAB for:
  - Parameter selection (e.g., frequency)
  - Range-Doppler maps
  - CFAR target detection visualization
  - 3D spatial positioning of detected targets
- **Multi-Frequency Analysis** from **1 GHz to 20 GHz** to evaluate performance variation across frequencies.
- **Advanced Signal Processing**: Non-coherent integration, beamforming, time-varying gain (TVG), FFT-based Doppler shift detection.
- **Utility** in domains such as:
  - Defence & Surveillance
  - Automotive ADAS systems
  - Aerospace monitoring
  - Remote sensing and environmental scanning
  - Educational radar experimentation

## Technologies Used

- MATLAB & Simulink
- Digital Signal Processing (DSP)
- Graphical User Interface (GUI) via App Designer
- Radar Range-Doppler Analysis
- CFAR (Constant False Alarm Rate) detection

## Codes
parameters code
 clc; clear; close all;

% Radar system parameters (Ensuring double precision)
range_res = double(50);  
max_range = double(10000);  
frequencies = double([1.5e9, 5e9, 8e9, 12e9, 30e9]);  
tx_gain = double(35);  
peak_power = double(20000.0);  
c = double(physconst('LightSpeed'));  

% Define radar structure template
paramRadarTemplate = struct( ...
    'fc', double(0), 'lambda', double(0), 'pulse_bw', double(0), 'pulse_length', double(0), ...
    'fs', double(0), 'noise_bw', double(0), 'num_pulse_int', double(0), 'prf', double(0), ...
    'range_bins', double([]), 'rng_res', double(0), 'DopplerFFTbins', double(0), ...
    'DopplerRes', double(0), 'DopplerOff', double(0), 'RCS', double([]), ...
    'targetPos', double([]), 'targetVel', double([]), 'timeRes', double(0), ...
    'max_range', double(0), 'tx_gain', double(0), 'peak_power', double(0));

% Create an array of structures
paramRadarConfigs = repmat(paramRadarTemplate, length(frequencies), 1);

% Updated target positions and velocities
target_positions = double([
    3200.50, 2500.00, 5200.80, 1800.90, 4600.20, 2100;  
    5800.75, 4500.00, 3500.20, 2800.55, 5300.10, 1500;  
    4400.25, 3100.00, 4700.60, 2300.33, 3500.50, 800   
]);

target_velocities = double([
    1500, -800, 1100, 950, -1400, 0;  
    -2000, 1200, -1500, 1000, 500, 0;  
    500, -300, 700, -600, 900, 0     
]);

target_RCS = double([12.0, 10.5, 15.2, 11.0, 9.8, 14.5]);

% Loop for each frequency
for fIdx = 1:length(frequencies)
    paramRadarVisual = paramRadarTemplate;  
    
    paramRadarVisual.fc = frequencies(fIdx);
    paramRadarVisual.lambda = c / paramRadarVisual.fc;  
    
    paramRadarVisual.pulse_bw = c / (2 * range_res);
    paramRadarVisual.pulse_length = 1 / paramRadarVisual.pulse_bw;
    paramRadarVisual.fs = 2 * paramRadarVisual.pulse_bw;
    paramRadarVisual.noise_bw = paramRadarVisual.pulse_bw;
    paramRadarVisual.num_pulse_int = 12;
    
    % Adjust PRF for better Doppler resolution
    if paramRadarVisual.fc <= 2e9  % L-band
        paramRadarVisual.prf = c / (5 * max_range);
    elseif paramRadarVisual.fc >= 20e9  % Ka-band
        paramRadarVisual.prf = c / (2.5 * max_range);
    else
        paramRadarVisual.prf = c / (3.5 * max_range);
    end

    fast_time = unigrid(0, 1 / paramRadarVisual.fs, 1 / paramRadarVisual.prf, '[)');
    paramRadarVisual.range_bins = c * fast_time / 2;
    paramRadarVisual.rng_res = c / (2 * paramRadarVisual.fs);

    paramRadarVisual.DopplerFFTbins = double(256);
    paramRadarVisual.DopplerRes = (paramRadarVisual.prf / paramRadarVisual.DopplerFFTbins) / 2;
    paramRadarVisual.DopplerOff = -paramRadarVisual.prf / 4;

    % Assign new target data
    paramRadarVisual.RCS = target_RCS;
    paramRadarVisual.targetPos = target_positions;
    paramRadarVisual.targetVel = target_velocities;

    paramRadarVisual.timeRes = 12 / paramRadarVisual.prf;
    paramRadarVisual.rng_res = double(50);  % FIX: Corrected field name to match the template
    
    % Assign missing fields
    paramRadarVisual.max_range = max_range;
    paramRadarVisual.tx_gain = tx_gain;
    paramRadarVisual.peak_power = peak_power;

    % Debug: Display field names before orderfields to confirm match
    fprintf('Checking field names for Frequency: %.2f GHz\n', frequencies(fIdx) / 1e9);
    disp(fieldnames(paramRadarVisual));
    disp(fieldnames(paramRadarTemplate));

    % Ensure identical structure before assignment
    paramRadarConfigs(fIdx) = orderfields(paramRadarVisual, paramRadarTemplate);
end

% Assign final configuration to base workspace
assignin('base', 'paramRadarConfigs', paramRadarConfigs);

% Apply CFAR processing
cfar_threshold = double(2.5);  
num_guard_cells = double(4);  
num_training_cells = double(12);

% Print target distances
fprintf('\nRadar Target Distances:\n');
for fIdx = 1:length(frequencies)
    fprintf('Frequency: %.2f GHz\n', frequencies(fIdx) / 1e9);
    targetPos = paramRadarConfigs(fIdx).targetPos;
    numTargets = size(targetPos, 2);
    for t = 1:numTargets
        range = norm(targetPos(:, t));  
        fprintf('Target %d: Position (%.2f, %.2f, %.2f) m, Range = %.2f m', ...
            t, targetPos(1,t), targetPos(2,t), targetPos(3,t), range);   
        if all(paramRadarConfigs(fIdx).targetVel(:, t) == 0)
            fprintf(' (Stationary Target)');
        end
        fprintf('\n');
    end
    fprintf('--------------------------------\n');
end
## 📈 Results & Observations

- High-resolution detection at **higher frequencies (10–20 GHz)** with trade-offs in signal stability and false alarms.
- **1 GHz and 3 GHz** offered better SNR stability and lower false alarm rates but at reduced resolution.
- GUI-based analysis helps visualize **target range, velocity, and spatial orientation**.
- Maximum radar range tested: **18 km**.

## 🔮 Future Scope

- Integration of **AI/ML** for intelligent target classification.
- Cognitive radar capabilities with environment-adaptive reconfiguration.
- Support for **mmWave/5G radar**, **edge computing**, and **real-time embedded processing** on FPGA/GPU.
- Augmented Reality (AR)-based radar visualization systems.

## 👩‍💻 Contributors

- **Shaik Karishma**
- **Dubasi Jhansi**  
Guided by **Prof. Dr. Prashant M. Menghal**, MCEME, Secunderabad

## 📅 Internship Timeline

- **Start Date:** 21st Jan, 2025  
- **Submission Date:** 29th March, 2025  
- **Duration:** 10 Weeks
