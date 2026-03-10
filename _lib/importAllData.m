%% Clear screen & data
% Work with importfile4 
function allData = importAllData(folderPath)
%% Define library
addpath('.\_lib\');
%% Import All Data
clc
%% 1. Klasör Yolu ve Metadata Tanımlaması
% folderPath = 'C:\_PhD\4_Tez\3_Tests\1_TestData\20260107\txtData';

% Tablodaki verileri Cell Array olarak tanımlıyoruz.
% Sütunlar: {FileName, Frequency, Waveform, Rate}
% Not: Sayısal değerler sayı, metin içerenler (örn: '30-60') string olarak girildi.
fileList = { ...
    'Hasan_PhD_RMC200_all_0_20260107_193412513', 0.5,      'Sine',            'Hz'; ...
    'Hasan_PhD_RMC200_all_0_20260107_193503929', 0.5,      'Sine',            'Hz'; ...
    'Hasan_PhD_RMC200_all_0_20260107_193718658', 1,        'Sine',            'Hz'; ...
    'Hasan_PhD_RMC200_all_0_20260107_193857313', 1.5,      'Sine',            'Hz'; ...
    'Hasan_PhD_RMC200_all_0_20260107_193948059', 2,        'Sine',            'Hz'; ...
    'Hasan_PhD_RMC200_all_0_20260107_194035435', 2.5,      'Sine',            'Hz'; ...
    'Hasan_PhD_RMC200_all_0_20260107_194118538', 3,        'Sine',            'Hz'; ...
    'Hasan_PhD_RMC200_all_0_20260107_194346986', '30-60',  'Triangle',        '30mmPs'; ...
    'Hasan_PhD_RMC200_all_0_20260107_194719861', 'NA',     'NA',              'NA'; ...
    'Hasan_PhD_RMC200_all_0_20260107_194736522', '30-60',  'Triangle',        '60mmPs'; ...
    'Hasan_PhD_RMC200_all_0_20260107_194837050', '-30+30', 'Triangle',        '60mmPs'; ...
    'Hasan_PhD_RMC200_all_0_20260107_195012267', '0.2-6',  'Frequency Sweep', '0.2-6' ...
};

%% 2. Yapıyı (Struct) Oluşturma Döngüsü
allData = struct(); % Boş struct oluştur

for i = 1:size(fileList, 1)
    
    % Listeden bilgileri çek
    currentFileName = fileList{i, 1};
    freqVal         = fileList{i, 2};
    waveType        = fileList{i, 3};
    rateVal         = fileList{i, 4};
    
    % Dosya yolunu hazırla
    fullFilePath = fullfile(folderPath, [currentFileName '.txt']);
    
    % Dosyanın varlığını kontrol et
    if isfile(fullFilePath)
        
        % Import fonksiyonunu çalıştır
        % [testData, t] çıktısı veriyor, biz 't' verisini alıyoruz.
        [~, t_data] = importfile4(fullFilePath);
        t_data.F_friction = t_data.F_piston_N - t_data.F_piston_from_a_N; 
        % allData yapısının içine kaydet
        % Struct field isimleri dinamik olarak dosya isminden oluşturuluyor
        allData.(currentFileName).Data      = t_data;
        allData.(currentFileName).Frequency = freqVal;
        allData.(currentFileName).Waveform  = waveType;
        allData.(currentFileName).Rate      = rateVal;
        
        fprintf('Eklendi: %s\n', currentFileName);
        
    else
        warning('Dosya bulunamadı: %s', fullFilePath);
        % Dosya yoksa bile metadata kısmını boş olarak oluşturabiliriz (Opsiyonel)
        % allData.(currentFileName).Data      = [];
        % allData.(currentFileName).Frequency = freqVal;
        % allData.(currentFileName).Waveform  = waveType;
        % allData.(currentFileName).Rate      = rateVal;
        % allData.(currentFileName).Status    = 'File Missing';
    end
end

disp('--- Tüm veriler allData yapısına aktarıldı ---');

%% 

end
