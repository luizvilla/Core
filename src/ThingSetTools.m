%
% Copyright (c) 2021-present LAAS-CNRS
%
%   This program is free software: you can redistribute it and/or modify
%   it under the terms of the GNU General Public License as published by
%   the Free Software Foundation, either version 2 of the License, or
%   (at your option) any later version.
%
%   This program is distributed in the hope that it will be useful,
%   but WITHOUT ANY WARRANTY; without even the implied warranty of
%   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
%   GNU General Public License for more details.
%
%   You should have received a copy of the GNU General Public License
%   along with this program.  If not, see <https://www.gnu.org/licenses/>.
%
% SPDX-License-Identifier: GPL-2.0-or-later
%
% @author Luiz Villa <luiz.villa@laas.fr>
%

classdef ThingSetTools < handle
    % ThingSetTools  Talk ThingSet Text Mode to a device over its
    % dedicated shell UART. MATLAB port of thingset_tools.py.
    %
    % Quick start:
    %
    %   ts = ThingSetTools();                   % port is optional: auto-
    %                                            % detected by USB VID/PID,
    %                                            % then a handshake probe
    %   tree = ts.discover();                   % walks the tree, writes
    %                                            % thingset_objects.json
    %   ts.read("Measurements/rV1Low_V")
    %   ts.write("Config", struct("wBlinkPeriod_s", 0.2))
    %
    % Auto-detection (see findPorts) first tries ports matching OwnTech's
    % USB vendor ID, falling back to every serial port on the system if
    % none match. Since a board's console and ThingSet-shell ports share
    % the same VID/PID, each candidate is actually opened and sent the
    % "select thingset" handshake - only the shell port responds.

    properties (SetAccess = private)
        Port            % connected serial port name, e.g. "/dev/ttyACM1"
        Tree            % struct built by discover(): name -> item/group metadata
    end

    properties
        % When true, every step (port candidates, connection attempts,
        % raw bytes written/read, response parsing) is printed to stderr
        % via log(). Toggle at any time: ts.Verbose = true;
        Verbose (1,1) logical = false
    end

    properties (Access = private)
        Serial          % underlying serialport object
        Discovered = false
    end

    properties (Constant)
        % OwnTech boards' USB vendor ID. Both the console and the
        % ThingSet-shell CDC-ACM ports of the same board share this VID
        % (and usually the same PID too), so it narrows the search but
        % doesn't single out the shell port by itself - see the
        % handshake-probing loop in the constructor.
        OwnTechUsbVid = "2FE3"

        % ThingSet response status codes (thingset.io/spec, "Access
        % Functions").
        StatusCodes = containers.Map( ...
            {hex2dec('81'), hex2dec('82'), hex2dec('84'), hex2dec('85'), ...
             hex2dec('A0'), hex2dec('A1'), hex2dec('A3'), hex2dec('A4'), ...
             hex2dec('A5'), hex2dec('A8'), hex2dec('A9'), hex2dec('AD'), ...
             hex2dec('AF'), hex2dec('C0'), hex2dec('C1'), hex2dec('C4'), ...
             hex2dec('C5')}, ...
            {'Created', 'Deleted', 'Changed', 'Content', ...
             'Bad Request', 'Unauthorized', 'Forbidden (read-only value)', ...
             'Not Found', 'Method Not Allowed', 'Request Entity Incomplete', ...
             'Conflict', 'Request Entity Too Large', ...
             'Unsupported Content-Format', 'Internal Server Error', ...
             'Not Implemented', 'Gateway Timeout', 'Not a Gateway'})
    end

    methods
        function obj = ThingSetTools(port, baudRate, timeoutSeconds, vid, pid, verbose)
            % Connect to a ThingSet-over-shell device. If `port` is
            % omitted, candidates are found by USB `vid`/`pid` (see
            % findPorts), falling back to every serial port on the system
            % if none match, and each is tried in turn until one answers
            % the ThingSet handshake.
            %
            % Pass verbose=true (or set ts.Verbose=true afterwards) to
            % print every step - candidate ports, connection attempts and
            % why each one failed, raw serial traffic, response parsing -
            % to stderr. Useful when the device is found on one machine
            % but not another.
            arguments
                port (1,1) string = ""
                baudRate (1,1) double = 115200
                timeoutSeconds (1,1) double = 1.0
                vid (1,1) string = ThingSetTools.OwnTechUsbVid
                pid string = ""
                verbose (1,1) logical = false
            end

            obj.Tree = struct();
            obj.Verbose = verbose;

            obj.log("MATLAB %s, serialport function available: %d", ...
                version(), exist("serialport", "file") > 0 || exist("serialport", "builtin") > 0);

            if strlength(port) > 0
                obj.log("explicit port given: %s", port);
                obj.connect(port, baudRate, timeoutSeconds);
                return
            end

            obj.log("searching for candidate ports (vid=%s, pid=%s)", vid, pid);
            candidates = ThingSetTools.findPorts(vid, pid, verbose);
            obj.log("VID/PID match: %d candidate(s): %s", numel(candidates), ...
                strjoin(candidates, ", "));
            if isempty(candidates)
                candidates = serialportlist("available");
                obj.log("no VID/PID match, falling back to all available ports: %d found: %s", ...
                    numel(candidates), strjoin(candidates, ", "));
            end
            for i = 1:numel(candidates)
                obj.log("trying candidate %d/%d: %s", i, numel(candidates), candidates(i));
                try
                    obj.connect(candidates(i), baudRate, timeoutSeconds);
                    obj.log("connected on %s", candidates(i));
                    return
                catch ME
                    obj.log("candidate %s failed: [%s] %s", candidates(i), ME.identifier, ME.message);
                end
            end
            error("ThingSetTools:notFound", ...
                "no ThingSet-over-shell device found (tried %d candidate port(s)). " + ...
                "Set verbose=true (e.g. ThingSetTools('', 115200, 1.0, '%s', '', true)) to see why each candidate failed.", ...
                numel(candidates), vid);
        end

        function close(obj)
            obj.Serial = [];
        end

        function delete(obj)
            obj.Serial = [];
        end

        % ---- ThingSet requests ---------------------------------------

        function value = get(obj, path)
            % GET a path. Returns the parsed JSON value (or a group dump).
            arguments
                obj
                path (1,1) string = ""
            end
            value = obj.parseResponse(obj.transact("?" + path, 3));
        end

        function names = fetchChildren(obj, path)
            % FETCH-with-null a group path; returns child names as a
            % string array.
            arguments
                obj
                path (1,1) string = ""
            end
            value = obj.parseResponse(obj.transact("?" + path + " null", 3));
            if iscell(value)
                % jsondecode's cell array shape isn't guaranteed to be a
                % row vector; force one so `for name = fetchChildren(...)`
                % iterates per-element rather than once over a column.
                names = reshape(string(value), 1, []);
            else
                names = string.empty;
            end
        end

        function value = read(obj, path)
            % Read a single item's value.
            value = obj.get(path);
        end

        function write(obj, path, values)
            % UPDATE `path` with `values` (a scalar struct of
            % item_name -> value). Handles the Zephyr shell's
            % unescaped-double-quote stripping automatically.
            payload = strrep(jsonencode(values), '"', '\"');
            obj.parseResponse(obj.transact("=" + path + " " + payload, 3));
        end

        % ---- discovery -------------------------------------------------

        function tree = discover(obj, jsonPath)
            % Recursively walk the device's ThingSet object tree, save it
            % to `jsonPath` (default "thingset_objects.json"; pass "" to
            % skip saving), and return it as a nested struct.
            arguments
                obj
                jsonPath (1,1) string = "thingset_objects.json"
            end
            tree = obj.discoverNode("");
            obj.Tree = tree;
            obj.Discovered = true;
            if strlength(jsonPath) > 0
                fid = fopen(jsonPath, 'w');
                fwrite(fid, jsonencode(tree, "PrettyPrint", true));
                fclose(fid);
            end
        end

        % ---- bulk read / write -----------------------------------------

        function values = readAll(obj, tree)
            % Read every readable item found by discover(), as a nested
            % struct mirroring the object tree. Calls discover() first if
            % needed.
            arguments
                obj
                tree = []
            end
            if isempty(tree)
                if ~obj.Discovered
                    tree = obj.discover();
                else
                    tree = obj.Tree;
                end
            end
            values = struct();
            names = fieldnames(tree);
            for i = 1:numel(names)
                name = names{i};
                meta = tree.(name);
                if strcmp(meta.Type, "group")
                    values.(name) = obj.readAll(meta.Children);
                elseif strcmp(meta.Type, "executable")
                    continue
                else
                    try
                        values.(name) = obj.read(meta.Path);
                    catch ME
                        values.(name) = "<error: " + string(ME.message) + ">";
                    end
                end
            end
        end

        function results = writeValues(obj, values)
            % Write `values` to the device's writable ("w"/"s") items.
            %
            % `values` may be a nested struct mirroring the discovered
            % tree (e.g. struct("Config", struct("wBlinkPeriod_s", 0.2)))
            % or a containers.Map with flat "Group/item" keys. Requires
            % discover() to have run first, so items can be checked
            % against their declared access type before anything is
            % sent. Returns a containers.Map of {path: "ok" or
            % "error: ..."}.
            if ~obj.Discovered
                obj.discover();
            end

            if isa(values, "containers.Map")
                flat = values;
            else
                flat = obj.flattenStruct(values, "");
            end

            results = containers.Map("KeyType", "char", "ValueType", "any");
            byGroup = containers.Map("KeyType", "char", "ValueType", "any");
            allPaths = keys(flat);
            for i = 1:numel(allPaths)
                path = allPaths{i};
                value = flat(path);
                meta = obj.lookup(path);
                if isempty(meta)
                    results(path) = "error: unknown ThingSet path";
                    continue
                end
                if ~any(strcmp(meta.Type, ["writable", "writable-setting"]))
                    results(path) = sprintf("error: %s is %s, not writable", path, meta.Type);
                    continue
                end
                idx = find(path == '/', 1, 'last');
                if isempty(idx)
                    parent = '';
                    leaf = path;
                else
                    parent = path(1:idx-1);
                    leaf = path(idx+1:end);
                end
                if ~isKey(byGroup, parent)
                    byGroup(parent) = struct();
                end
                grp = byGroup(parent);
                grp.(leaf) = value;
                byGroup(parent) = grp;
            end

            groupNames = keys(byGroup);
            for i = 1:numel(groupNames)
                parent = groupNames{i};
                leaves = byGroup(parent);
                leafNames = fieldnames(leaves);
                try
                    obj.write(parent, leaves);
                    for j = 1:numel(leafNames)
                        results(obj.joinPath(parent, leafNames{j})) = "ok";
                    end
                catch ME
                    for j = 1:numel(leafNames)
                        results(obj.joinPath(parent, leafNames{j})) = "error: " + string(ME.message);
                    end
                end
            end
        end
    end

    methods (Static)
        function ports = findPorts(vid, pid, verbose)
            % List serial ports matching a USB vendor ID (and optionally
            % a specific product ID). Defaults to OwnTech's VID. Reads
            % /sys/class/tty on Linux, WMI (Win32_PnPEntity) on Windows;
            % falls back to returning every available port elsewhere
            % (e.g. macOS). Adapted from old/old4/find_devices.py, whose
            % Python pyserial `list_ports.comports()` gets VID/PID for
            % free on all three platforms - this reimplements just the
            % Linux and Windows cases.
            %
            % Pass verbose=true to print, per port, the VID/PID found (or
            % why it couldn't be read) to stderr.
            arguments
                vid (1,1) string = ThingSetTools.OwnTechUsbVid
                pid string = ""
                verbose (1,1) logical = false
            end
            allPorts = serialportlist("available");
            if verbose
                ThingSetTools.printLog("findPorts: %d port(s) reported by serialportlist: %s", ...
                    numel(allPorts), strjoin(allPorts, ", "));
            end
            winMap = containers.Map("KeyType", "char", "ValueType", "any");
            if ispc
                winMap = ThingSetTools.readUsbIdsWindows(verbose);
            end
            ports = string.empty;
            for i = 1:numel(allPorts)
                info = ThingSetTools.readUsbIds(allPorts(i), verbose, winMap);
                if isempty(info)
                    continue
                end
                match = strcmpi(info.vid, vid) && (strlength(pid) == 0 || strcmpi(info.pid, pid));
                if verbose
                    ThingSetTools.printLog("findPorts: %s vid=%s pid=%s -> %s", ...
                        allPorts(i), info.vid, info.pid, string(match));
                end
                if match
                    ports(end+1) = allPorts(i); %#ok<AGROW>
                end
            end
        end
    end

    methods (Static, Access = private)
        function s = hexPreview(bytes)
            % Render up to the first 64 received bytes as hex + a
            % printable-ASCII rendering, for RX log lines. Long
            % responses are truncated so the log stays readable.
            n = numel(bytes);
            shown = bytes(1:min(n, 64));
            hexPart = strjoin(compose("%02X", shown), " ");
            printable = shown;
            printable(printable < 32 | printable > 126) = uint8('.');
            asciiPart = string(char(printable));
            s = hexPart + "  |" + asciiPart + "|";
            if n > 64
                s = s + sprintf(" (+%d more byte(s))", n - 64);
            end
        end

        function printLog(fmt, varargin)
            % Shared stderr-logging sink for both instance methods
            % (via log()) and the Static discovery helpers, which have
            % no `obj` to read Verbose from.
            fprintf(2, "[ThingSetTools %s] " + fmt + "\n", ...
                datestr(now, 'HH:MM:SS.FFF'), varargin{:}); %#ok<TNOW1,DATST>
        end

        function info = readUsbIds(portName, verbose, winMap)
            % Best-effort USB VID/PID lookup: Linux sysfs, or a
            % pre-fetched Windows WMI map (see readUsbIdsWindows).
            % Returns [] if unavailable (unsupported platform, or not a
            % USB-CDC device).
            arguments
                portName (1,1) string
                verbose (1,1) logical = false
                winMap = containers.Map("KeyType", "char", "ValueType", "any")
            end
            info = [];
            if isunix
                [~, devName] = fileparts(portName);
                base = "/sys/class/tty/" + devName + "/device/../";
                vidFile = base + "idVendor";
                pidFile = base + "idProduct";
                if isfile(vidFile) && isfile(pidFile)
                    info = struct( ...
                        "vid", strtrim(fileread(vidFile)), ...
                        "pid", strtrim(fileread(pidFile)));
                elseif verbose
                    ThingSetTools.printLog("readUsbIds: %s has no %s/%s (not a USB-CDC port, or sysfs layout differs on this machine)", ...
                        portName, vidFile, pidFile);
                end
            elseif ispc
                key = char(portName);
                if isKey(winMap, key)
                    info = winMap(key);
                elseif verbose
                    ThingSetTools.printLog("readUsbIds: no USB VID/PID found for %s via WMI (not a USB-CDC device, or Win32_PnPEntity didn't report it)", ...
                        portName);
                end
            elseif verbose
                ThingSetTools.printLog("readUsbIds: VID/PID lookup not implemented on this platform (%s)", computer);
            end
        end

        function map = readUsbIdsWindows(verbose)
            % Query all USB-attached COM ports' VID/PID in one shot via
            % WMI/PowerShell (one process launch for every port, not one
            % per port - that would be far too slow). Windows has no
            % sysfs equivalent: VID/PID lives inside the PnP DeviceID
            % string, e.g. "USB\VID_2FE3&PID_0100\6&1234...".
            arguments
                verbose (1,1) logical = false
            end
            map = containers.Map("KeyType", "char", "ValueType", "any");
            cmd = ['powershell -NoProfile -Command "Get-CimInstance Win32_PnPEntity | ' ...
                'Where-Object { $_.Name -match ''\(COM[0-9]+\)'' } | ' ...
                'Select-Object Name, DeviceID | ConvertTo-Json -Compress"'];
            [status, out] = system(cmd);
            if status ~= 0 || strlength(strtrim(string(out))) == 0
                if verbose
                    ThingSetTools.printLog("readUsbIdsWindows: WMI query failed (status=%d): %s", status, out);
                end
                return
            end
            try
                entries = jsondecode(out);
            catch ME
                if verbose
                    ThingSetTools.printLog("readUsbIdsWindows: failed to parse WMI output as JSON: %s", ME.message);
                end
                return
            end
            for i = 1:numel(entries)
                e = entries(i);
                nameTok = regexp(e.Name, '\((COM\d+)\)', 'tokens', 'once');
                idTok = regexp(e.DeviceID, 'VID_([0-9A-Fa-f]{4})&PID_([0-9A-Fa-f]{4})', 'tokens', 'once');
                if isempty(nameTok) || isempty(idTok)
                    continue
                end
                map(nameTok{1}) = struct("vid", string(idTok{1}), "pid", string(idTok{2}));
            end
            if verbose
                ThingSetTools.printLog("readUsbIdsWindows: found VID/PID for %d of the queried COM port(s)", map.Count);
            end
        end

        function path = joinPath(parent, leaf)
            % Note: isempty("") is false for a MATLAB string scalar (it's
            % a 1x1 array whose one element happens to be empty text) -
            % only isempty('') on a char array is true. Since callers
            % pass both char (containers.Map keys) and string (path
            % building during discovery), strlength(string(...)) is the
            % only check that's correct for either input type.
            if strlength(string(parent)) == 0
                path = char(leaf);
            else
                path = [char(parent) '/' char(leaf)];
            end
        end
    end

    methods (Access = private)
        function log(obj, fmt, varargin)
            % Print an instrumentation line to stderr when Verbose is
            % on. Timestamped so serial-timing issues (slow USB
            % enumeration, late device response) are visible.
            if ~obj.Verbose
                return
            end
            ThingSetTools.printLog(fmt, varargin{:});
        end

        function connect(obj, port, baudRate, timeoutSeconds)
            obj.log("opening %s at %d baud, timeout=%.2fs", port, baudRate, timeoutSeconds);
            obj.Port = port;
            % FlowControl is pinned to "none" (not just left at its
            % default) because some Windows COM ports - notably
            % Bluetooth-over-serial and other virtual ports - can enable
            % hardware flow control at the driver level. When that
            % happens, write() blocks forever waiting for CTS that never
            % comes, and MATLAB's Timeout property does not bound that
            % wait (it only governs read()). Explicit "none" avoids the
            % hang outright rather than relying on the port's default.
            obj.Serial = serialport(port, baudRate, "Timeout", timeoutSeconds, "FlowControl", "none");
            pause(0.3);
            flush(obj.Serial);
            obj.log("sending blank line to clear any partial input");
            obj.transact("", timeoutSeconds);
            obj.log("sending handshake: select thingset");
            obj.transact("select thingset", timeoutSeconds);
            obj.get("");
            obj.log("handshake ok, root GET succeeded");
        end

        function tree = discoverNode(obj, path)
            tree = struct();
            names = obj.fetchChildren(path);
            for i = 1:numel(names)
                name = names(i);
                childPath = obj.joinPath(path, name);
                value = obj.get(childPath);
                fieldName = matlab.lang.makeValidName(name);
                if isstruct(value)
                    tree.(fieldName) = struct( ...
                        "Type", "group", ...
                        "Path", childPath, ...
                        "Children", obj.discoverNode(childPath));
                else
                    tree.(fieldName) = struct( ...
                        "Type", obj.classifyLeaf(name), ...
                        "Path", childPath);
                end
            end
        end

        function kind = classifyLeaf(~, name)
            chars = char(name);
            if isempty(chars)
                kind = "unknown";
                return
            end
            switch chars(1)
                case 'r'
                    kind = "read-only";
                case 'w'
                    kind = "writable";
                case 's'
                    kind = "writable-setting";
                case 'x'
                    kind = "executable";
                otherwise
                    kind = "informational";
            end
        end

        function meta = lookup(obj, path)
            parts = strsplit(path, '/');
            container = obj.Tree;
            meta = [];
            for i = 1:numel(parts)
                name = matlab.lang.makeValidName(parts{i});
                if ~isstruct(container) || ~isfield(container, name)
                    meta = [];
                    return
                end
                meta = container.(name);
                if i < numel(parts)
                    if ~isfield(meta, "Children")
                        meta = [];
                        return
                    end
                    container = meta.Children;
                end
            end
        end

        function flat = flattenStruct(obj, s, prefixPath)
            flat = containers.Map("KeyType", "char", "ValueType", "any");
            names = fieldnames(s);
            for i = 1:numel(names)
                name = names{i};
                path = obj.joinPath(prefixPath, name);
                value = s.(name);
                if isstruct(value)
                    sub = obj.flattenStruct(value, path);
                    subKeys = keys(sub);
                    for j = 1:numel(subKeys)
                        flat(subKeys{j}) = sub(subKeys{j});
                    end
                else
                    flat(path) = value;
                end
            end
        end

        function body = transact(obj, cmd, timeoutSeconds)
            flush(obj.Serial);
            obj.log("TX: %s", cmd);
            write(obj.Serial, uint8([char(cmd) 13 10]), "uint8");

            ansiPat = [char(27) '\[[0-9;]*m'];
            promptPat = '[A-Za-z0-9_-]+:~\$\s*';

            t0 = tic;
            raw = uint8([]);
            plainText = "";
            found = false;
            while toc(t0) < timeoutSeconds
                n = obj.Serial.NumBytesAvailable;
                if n > 0
                    chunk = read(obj.Serial, n, "uint8");
                    obj.log("RX %d byte(s) after %.3fs: %s", n, toc(t0), ...
                        ThingSetTools.hexPreview(chunk));
                    raw = [raw, chunk]; %#ok<AGROW>
                    plainText = string(regexprep(char(raw), ansiPat, ''));
                    if ~isempty(regexp(plainText, promptPat, 'once'))
                        found = true;
                        break
                    end
                else
                    pause(0.02);
                end
            end
            if ~found
                obj.log("TIMEOUT after %.2fs waiting for prompt matching /%s/; %d byte(s) received: %s", ...
                    timeoutSeconds, promptPat, numel(raw), ThingSetTools.hexPreview(raw));
                error("ThingSetTools:timeout", "no response to '%s' on %s", cmd, obj.Port);
            end
            obj.log("prompt matched after %.3fs, %d byte(s) total", toc(t0), numel(raw));

            body = regexprep(plainText, promptPat, '');
            lines = strsplit(body, {char(13), newline});
            cmdTrim = strtrim(cmd);
            keepLines = string.empty;
            for i = 1:numel(lines)
                l = strtrim(string(lines{i}));
                if strlength(l) > 0 && l ~= cmdTrim
                    keepLines(end+1) = l; %#ok<AGROW>
                end
            end
            body = strtrim(strjoin(keepLines, newline));
            obj.log("RX body: %s", body);
        end

        function value = parseResponse(obj, body)
            % Note: the payload capture group must NOT be nested inside
            % an optional non-capturing group - MATLAB's regexp silently
            % drops it from 'tokens' output in that case (verified
            % against R2024b). \s*(.*) always matches (empty when there
            % is no payload), so no optional wrapper is needed.
            tok = regexp(char(body), ':([0-9A-Fa-f]{2})\s*(.*)$', 'tokens', 'once');
            if isempty(tok)
                obj.log("parseResponse: no status code found in body: %s", body);
                error("ThingSetTools:badResponse", "unexpected response: %s", body);
            end
            code = hex2dec(tok{1});
            payloadStr = string(tok{2});
            obj.log("parseResponse: status=0x%02X payload=%s", code, payloadStr);
            if code >= hex2dec('A0')
                if isKey(obj.StatusCodes, code)
                    msg = obj.StatusCodes(code);
                else
                    msg = 'Unknown error';
                end
                error("ThingSetTools:status", "0x%02X %s", code, msg);
            end
            if strlength(payloadStr) == 0
                value = [];
                return
            end
            try
                value = jsondecode(char(payloadStr));
            catch ME
                obj.log("parseResponse: jsondecode failed (%s), returning raw string", ME.message);
                value = payloadStr;
            end
        end
    end
end
