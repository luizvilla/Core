function port = findShieldDevicePort(varargin)
    % Auto-discover the Twist/Ownverter shield's serial port by USB
    % VID/PID, falling back to a manual serialportlist selection when
    % autodetection finds zero or more than one candidate.

    p = inputParser;
    addParameter(p, 'VendorID', '2fe3');
    addParameter(p, 'ProductID', '0101');
    addParameter(p, 'Interactive', true);
    parse(p, varargin{:});

    vid = lower(p.Results.VendorID);
    pid = lower(p.Results.ProductID);

    if isunix && ~ismac
        candidates = findPortsLinux(vid, pid);
    elseif ispc
        candidates = findPortsWindows(vid, pid);
    else
        candidates = {};
    end

    if isscalar(candidates)
        port = candidates{1};
        return;
    elseif numel(candidates) > 1
        warning('ShieldDevice:MultipleDevices', ...
            'Found %d candidate ports matching VID %s / PID %s; falling back to manual selection.', ...
            numel(candidates), vid, pid);
    else
        warning('ShieldDevice:DeviceNotFound', ...
            'No device matching VID %s / PID %s was auto-detected; falling back to manual selection.', ...
            vid, pid);
    end

    port = manualPortSelection(p.Results.Interactive);
end

function ports = findPortsLinux(vid, pid)
    ports = {};
    ttyRoot = '/sys/class/tty';
    if ~isfolder(ttyRoot)
        return
    end
    entries = dir(ttyRoot);
    for i = 1:numel(entries)
        name = entries(i).name;
        if startsWith(name, '.')
            continue
        end
        devicePath = fullfile(ttyRoot, name, 'device');
        if ~isfolder(devicePath)
            continue
        end
        vidFile = findParentFile(devicePath, 'idVendor');
        pidFile = findParentFile(devicePath, 'idProduct');
        if isempty(vidFile) || isempty(pidFile)
            continue
        end
        actualVid = strtrim(fileread(vidFile));
        actualPid = strtrim(fileread(pidFile));
        if strcmpi(actualVid, vid) && strcmpi(actualPid, pid)
            ports{end + 1} = fullfile('/dev', name); %#ok<AGROW>
        end
    end
end

function found = findParentFile(startDir, filename)
    % Walk up from startDir looking for filename, since idVendor/idProduct
    % live on the USB device node while /sys/class/tty/<tty>/device
    % typically resolves to a child interface node.
    found = '';
    d = char(java.io.File(startDir).getCanonicalPath());
    for depth = 1:6
        candidate = fullfile(d, filename);
        if isfile(candidate)
            found = candidate;
            return
        end
        parent = fileparts(d);
        if strcmp(parent, d) || isempty(parent)
            break
        end
        d = parent;
    end
end

function ports = findPortsWindows(vid, pid)
    ports = {};
    idPattern = sprintf('VID_%s&PID_%s', upper(vid), upper(pid));
    [status, cmdout] = system('wmic path Win32_PnPEntity get DeviceID,Name /format:list');
    if status ~= 0
        return
    end
    blocks = regexp(cmdout, '\r?\n\r?\n', 'split');
    for i = 1:numel(blocks)
        block = blocks{i};
        if contains(upper(block), idPattern)
            tok = regexp(block, 'COM(\d+)', 'tokens', 'once');
            if ~isempty(tok)
                ports{end + 1} = ['COM' tok{1}]; %#ok<AGROW>
            end
        end
    end
end

function port = manualPortSelection(interactive)
    available = serialportlist("available");
    if isempty(available)
        error('ShieldDevice:NoPortsAvailable', 'No serial ports available for manual selection.');
    end

    if ~interactive
        if isscalar(available)
            port = char(available(1));
            return
        end
        error('ShieldDevice:AmbiguousSelection', ...
            'Multiple ports available and Interactive is false: %s', strjoin(cellstr(available), ', '));
    end

    fprintf('Available serial ports:\n');
    for i = 1:numel(available)
        fprintf('  [%d] %s\n', i, available(i));
    end
    choice = input(sprintf('Select port [1-%d]: ', numel(available)));
    if isempty(choice) || ~isnumeric(choice) || choice < 1 || choice > numel(available)
        error('ShieldDevice:InvalidSelection', 'Invalid selection.');
    end
    port = char(available(choice));
end
