function plotData(log)
% The function plots the data from the log.

    if isempty(log)
        warning('No data inside the log.');
        return;
    end
    
    t = [log.t].';

    % Exemplo de acesso ao primeiro joint:
    q1 = cell2mat(arrayfun(@(e) e.q(1), log, 'UniformOutput', false)).';

    figure;
    plot(t, q1);
    xlabel('Tempo [s]');
    ylabel('q_1 [rad]');
    title('Posição da junta 1 ao longo do tempo');
    grid on;
end
