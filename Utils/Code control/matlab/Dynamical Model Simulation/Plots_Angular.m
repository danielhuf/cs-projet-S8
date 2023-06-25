

% Plotting
figure;
hold on;
for i = 1:3
    plot(t, qref(:, i), 'b-', 'LineWidth', 1.5);
    plot(t, q_ss(:, i), 'r--', 'LineWidth', 1.5);
end
hold off;
xlabel('t');
ylabel('q_{ref}, y_{ss}');
legend('q_{ref} 1', 'q_{ss} 1', 'q_{ref} 2', 'q_{ss} 2', 'q_{ref} 3', 'q_{ss} 3');
title('Plot of the Angular Position reference and the actual Angular Position (State Space Model)');
grid on;

figure;
hold on;
for i = 1:3
    plot(t, q_ref(:, i), 'b-', 'LineWidth', 1.5);
    plot(t, q(:, i), 'r--', 'LineWidth', 1.5);
end
hold off;
xlabel('t');
ylabel('q_{ref}, q');
legend('q_{ref} 1', 'q1', 'q_{ref} 2', 'q2', 'q_{ref} 3', 'q3');
title('Plot of the Angular Position reference and the actual Angular Position (Non Linear Model)');
grid on;


