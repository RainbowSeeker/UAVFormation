
log = importdata("formation_log.txt");
log_data = log.data;

plot(log_data(:, 2), log_data(:, 5), ...
     log_data(:, 3), log_data(:, 6), ...
     log_data(:, 4), log_data(:, 7));