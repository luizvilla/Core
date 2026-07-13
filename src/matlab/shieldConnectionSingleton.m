function device = shieldConnectionSingleton(action, device)
    % Internal state holder shared by getShieldConnection/releaseShieldConnection.
    % Not intended to be called directly -- MATLAB `persistent` variables are
    % scoped per-function, so this exists purely to give those two independent
    % top-level functions a common place to store/retrieve/clear the same
    % ShieldDevice handle.
    %
    % action: 'get' returns the stored handle (or [] if none); 'set' stores
    % device and returns it; 'clear' clears the stored handle and returns [].

    persistent storedDevice

    if nargin < 2
        device = [];
    end

    switch action
        case 'get'
            device = storedDevice;
        case 'set'
            storedDevice = device;
        case 'clear'
            storedDevice = [];
            device = [];
        otherwise
            error('shieldConnectionSingleton:InvalidAction', 'Invalid action: %s', action);
    end
end
