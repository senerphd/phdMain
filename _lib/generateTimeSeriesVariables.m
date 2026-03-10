function [] = generateTimeSeriesVariables(t)
% t struct'ındaki alan isimlerini al
fieldNames = fieldnames(t);

% Zaman vektörünü sabitle (t.t)
if isfield(t, 't')
    timeVec = t.t;
else
    error('Struct içinde "t" isimli zaman verisi bulunamadı.');
end

% Her bir alan için döngü
for i = 1:length(fieldNames)
    varName = fieldNames{i};
    
    % Zaman değişkeninin (t.t) kendisini atla, onu zaten referans olarak kullanıyoruz
    if strcmp(varName, 't')
        continue;
    end
    
    % Veriyi çek
    dataVec = t.(varName);
    
    % Boyut kontrolü (Zaman ve Veri boyutu eşleşmeli)
    if length(dataVec) == length(timeVec)
        
        % 1. Timeseries nesnesini oluştur
        ts = timeseries(dataVec, timeVec);
        ts.Name = varName; % Plot başlıklarında görünmesi için
        
        % 2. Workspace'e aynı isimle değişken olarak at
        assignin('base', varName, ts);
        
        fprintf('%s \t \t değişkeni timeseries olarak oluşturuldu.\n', varName);
    end
end
end
