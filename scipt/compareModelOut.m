% Plot Model Output
ins = get(out, 'INS');
plot3(ins.x_R.Data(:, 1), ins.y_R.Data(:, 1), ins.h_R.Data(:, 1), ...
      ins.x_R.Data(:, 2), ins.y_R.Data(:, 2), ins.h_R.Data(:, 2), ...
      ins.x_R.Data(:, 3), ins.y_R.Data(:, 3), ins.h_R.Data(:, 3));


% PN = get(out.logsout, 'PN').Values.Data;
% PE = get(out.logsout, 'PE').Values.Data;
% plot(PN,PE);
