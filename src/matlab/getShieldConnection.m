function device = getShieldConnection(varargin)
    % Returns a shared ShieldDevice connection, creating it (discovery + the
    % one-time setup sequence) on first call. Subsequent calls -- from this or
    % any other function/block in the same MATLAB session -- return the same
    % handle without repeating discovery or setup. Intended to be called from
    % Simulink System object setupImpl methods (ShieldSendBlock/ShieldGetBlock)
    % so both blocks share one open serial connection.
    %
    % Name-value options (all optional):
    %   VendorID, ProductID, Interactive - forwarded to findShieldDevicePort.
    %     Interactive defaults to false here (unlike findShieldDevicePort's own
    %     default of true), since this is meant to run unattended from a
    %     Simulink block rather than from a human at a terminal; pass
    %     'Interactive', true to get the manual-selection prompt instead.
    %   ForcedPort - if non-empty, skip auto-discovery entirely and open this
    %     port directly. Used by no-hardware tests to inject a pty loopback
    %     port instead of relying on real USB auto-discovery.

    p = inputParser;
    addParameter(p, 'VendorID', '2fe3');
    addParameter(p, 'ProductID', '0101');
    addParameter(p, 'Interactive', false);
    addParameter(p, 'ForcedPort', '');
    parse(p, varargin{:});

    device = shieldConnectionSingleton('get');
    if ~isempty(device) && isvalid(device)
        return
    end

    if isempty(p.Results.ForcedPort)
        port = findShieldDevicePort('VendorID', p.Results.VendorID, ...
            'ProductID', p.Results.ProductID, 'Interactive', p.Results.Interactive);
    else
        port = p.Results.ForcedPort;
    end

    device = ShieldDevice(port);
    device.sendCommand('IDLE');
    device.sendCommand('BUCK', 'LEG1', 'ON');
    device.sendCommand('BUCK', 'LEG2', 'ON');
    device.sendCommand('LEG', 'LEG1', 'ON');
    device.sendCommand('LEG', 'LEG2', 'ON');
    device.sendCommand('REFERENCE', 'LEG1', 'V1', 5);
    device.sendCommand('POWER_ON');

    shieldConnectionSingleton('set', device);
end
