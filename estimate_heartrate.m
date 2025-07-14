clc;
clear;

% Feather M0 보드의 IP 주소로 변경
ipAddress = '192.168.35.175';  % Feather M0 보드의 실제 IP 주소
port = 80;

% TCP/IP 설정
t = tcpclient(ipAddress, port);

% 데이터 수신 설정
duration = 10;  % 수신할 시간 (초)

% Stop 버튼 설정
hFig = figure('Name', 'Heart Rate Monitor', 'NumberTitle', 'off');
uicontrol('Style', 'pushbutton', 'String', 'Stop', ...
          'Position', [20 20 50 20], ...
          'Callback', 'setappdata(gcf, ''stop'', 1)');

setappdata(hFig, 'stop', 0);

bpmData = [];

% 데이터 수신
while ~getappdata(hFig, 'stop')
    data = [];  
    startTime = tic;
    while toc(startTime) < duration
        if t.BytesAvailable > 0
            rawData = readline(t);
            values = str2num(rawData); 
            if length(values) == 4
                data = [data; values]; 
            else
                disp(['Unexpected data format: ', rawData]); 
            end
        end
    end

    % 수신된 데이터 크기 및 샘플링 레이트 추정
    numSamples = size(data, 1);
    sampleRate = numSamples / duration;  % 샘플링 레이트 추정

    % IIR 필터 적용
    red_signal = data(:, 1);
    red_signal = red_signal - mean(red_signal); 
    [b, a] = butter(5, [0.5 4] / (sampleRate / 2), 'bandpass');  % 대역 통과 필터 
    filtered_red = filtfilt(b, a, red_signal);

    % 필터링된 PPG 신호 플롯
    figure;
    subplot(2,1,1);
    plot((0:numSamples-1)/sampleRate, red_signal);
    title('Original Red PPG Signal');
    xlabel('Time (s)');
    ylabel('Amplitude');

    subplot(2,1,2);
    plot((0:numSamples-1)/sampleRate, filtered_red);
    title('Filtered Red PPG Signal');
    xlabel('Time (s)');
    ylabel('Amplitude');

    % 가속도 데이터 설정
    accelData = data(:, 2:4);

    % NaN 및 Inf 값 제거 후 대체
    validIdx = all(~isnan(accelData) & ~isinf(accelData), 2);
    if sum(validIdx) < 10 % 데이터가 너무 적은 경우 처리
        disp('Insufficient valid data after removing NaN/Inf values. Skipping iteration.');
        continue;
    end
    accelData = accelData(validIdx, :);
    filtered_red = filtered_red(validIdx);
    numSamples = size(accelData, 1);

    % 가속도 데이터 정규화
    accelData = (accelData - mean(accelData)) ./ std(accelData);

    % 전체 데이터에서 최적의 Lambda 값을 찾기 위해 교차 검증 수행
    accelDataReshaped = reshape(accelData(1:numSamples, :), numSamples, 3);
    filteredRedReshaped = filtered_red(1:numSamples);
    % 교차 검증 횟수를 줄임 (데이터 부족 상황 고려)
    numCV = min(5, numSamples); % 교차 검증 횟수를 데이터 개수에 맞게 조정
    [lassoCoeff, FitInfo] = lasso(accelDataReshaped, filteredRedReshaped, 'CV', numCV, 'Standardize', true, 'MaxIter', 1e6, 'RelTol', 1e-4);
    idxLambdaMinMSE = FitInfo.IndexMinMSE;
    bestLambda = FitInfo.Lambda(idxLambdaMinMSE);

    disp(['Optimal Lambda (overall): ', num2str(bestLambda)]);

    % MA 추정 및 제거
    windowLength = 5;
    windowSamples = round(sampleRate * windowLength);
    numWindows = floor(numSamples / windowSamples);
    M = zeros(numSamples, 1);

    rSquaredScores = []; % R-squared 점수를 저장하기 위한 배열 초기화

    for i = 1:numWindows
        startIdx = (i-1) * windowSamples + 1;
        endIdx = min(i * windowSamples, numSamples);
        
        if endIdx - startIdx + 1 ~= windowSamples
            continue; 
        end
        
        A = accelData(startIdx:endIdx, :);
        B = filtered_red(startIdx:endIdx);
        
        % Lasso 적용
        X = lasso(A, B, 'Lambda', bestLambda, 'MaxIter', 1e6, 'RelTol', 1e-4, 'Standardize', true);
        B_pred = A * X;
        M(startIdx:endIdx) = B_pred;
        
        % R-squared 계산
        SS_res = sum((B - B_pred).^2);
        SS_tot = sum((B - mean(B)).^2);
        if SS_tot ~= 0
            rSquaredScores = [rSquaredScores; 1 - (SS_res / SS_tot)];
        else
            rSquaredScores = [rSquaredScores; NaN]; % NaN으로 추가하여 나중에 평균 계산 시 무시되도록 함
        end
    end

    % NaN 값 제외한 평균 R-squared 출력
    disp('R-squared scores for each window:');
    disp(rSquaredScores(~isnan(rSquaredScores)));

    % 새로운 PPG 근사치
    ppg_approx = filtered_red - M;

    % 필터링 후 및 MA 제거 후 PPG 신호 플롯
    figure;
    subplot(2,1,1);
    plot((0:numSamples-1)/sampleRate, filtered_red);
    title('Filtered Red PPG Signal');
    xlabel('Time (s)');
    ylabel('Amplitude');

    subplot(2,1,2);
    plot((0:numSamples-1)/sampleRate, ppg_approx);
    title('Denoised PPG Signal');
    xlabel('Time (s)');
    ylabel('Amplitude');

    % PPG 신호에서 피크 탐지 및 BPM 계산
    [pks, locs] = findpeaks(ppg_approx, 'MinPeakDistance', sampleRate/2); % 최소 피크 간격 설정
    RR_intervals = diff(locs) / sampleRate; % RR 간격 (초 단위)
    BPM = 60 ./ RR_intervals; % BPM 계산
    estimatedHR_peaks = mean(BPM); % 평균 BPM

    % 피크 탐지 결과 플롯
    figure;
    plot((0:numSamples-1)/sampleRate, ppg_approx);
    hold on;
    plot(locs/sampleRate, ppg_approx(locs), 'r*');
    title('Denoised PPG Signal with Peaks');
    xlabel('Time (s)');
    ylabel('Amplitude');
    hold off;

    % 피크 탐지 결과 출력
    disp(['Estimated Heart Rate (from peaks): ', num2str(estimatedHR_peaks), ' bpm']);
    % BPM 기록
    bpmData = [bpmData; estimatedHR_peaks];
end

% 데이터 csv 파일로 저장
csvwrite('heart_rate_data.csv', bpmData);

% TCP 연결 종료
clear t;

% GUI 창 닫기
close(hFig);
