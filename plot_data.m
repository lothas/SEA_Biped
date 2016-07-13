data = load_data_from_txt('pyno_data.txt');

figure
plot(data{1},data{2})
hold on
plot(data{1},data{3})
plot(data{1},data{4})
legend('raw','diff','filtered')
xlabel('time [millis]')

raw_data = data{2};
disp(['Min raw data: ',num2str(min(raw_data))]);
disp(['Max raw data: ',num2str(max(raw_data))]);

diff_raw_data = diff(raw_data);
diff_to_jump = mean(abs(diff_raw_data)) + std(abs(diff_raw_data));
ids = find(abs(diff_raw_data) > diff_to_jump);
scatter(data{1}(ids), raw_data(ids));

disp(['Diff expected to cause jump: ',num2str(max(diff_to_jump))]);
