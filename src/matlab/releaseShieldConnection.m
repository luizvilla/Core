function releaseShieldConnection()
    % Parks the shared ShieldDevice connection (sends IDLE) and clears it.
    % Safe to call multiple times, or when no connection exists -- only the
    % first call after a connection was created sends IDLE; later calls are
    % no-ops. Intended to be called from Simulink System object releaseImpl
    % methods (ShieldSendBlock/ShieldGetBlock).

    device = shieldConnectionSingleton('get');
    if ~isempty(device) && isvalid(device)
        try
            device.sendCommand('IDLE');
        catch
        end
    end
    shieldConnectionSingleton('clear');
end
