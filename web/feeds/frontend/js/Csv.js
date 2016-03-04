define(function () {
    function Csv(options, data) {
        this.options = options;
        this.init(data);
    };
    function parse_date(t) {
        t = t.slice(0, 4) + '-' + t.slice(4, 6) + '-' + t.slice(6, 11) + ':' + t.slice(11, 13) + ':' + t.slice(13);
        return new Date(t);
    }
    var type_size = {
        b: 1,
        ub: 1,
        w: 2,
        uw: 2,
        i: 4,
        ui: 4,
        l: 8,
        ul: 8,
        c: 1,
        f: 4,
        d: 8,
        t: 8,
        lt: 12
    }
    function TypeInfo(type, length) {
        this.type = type;
        this.length = length;
    }
    function parse(format, is_binary) {
        if (!format) {
            return;
        }
        if (format.constructor === String) {
            format = format.split(',');
        }
        var formats = [];
        for (var i in format) {
            var f = format[i];
            if (!f) {
                if (is_binary) {
                    throw new Error('expect full binary format');
                }
                formats.push(undefined);
                continue;
            }
            var match = f.match(/^([0-9]*)([a-z]+)$/);
            if (!match) {
                match = f.match(/^s\[([0-9]+)\]$/);
                if (!match) {
                    throw new Error('invalid format: ' + f);
                }
                var length = Number(match[1]);
                formats.push(new TypeInfo('s', length));
            } else {
                var count = match[1] ? Number(match[1]) : 1;
                var type = match[2];
                if (is_binary && type == 's') {
                    throw new Error('variable size string in binary not supported');
                }
                if (!(type in type_size) && type != 's') {
                    throw new Error('invalid type: ' + type + '; must be one of ' + Object.keys(type_size));
                }
                for (var j = 0; j < count; ++j) {
                    formats.push(new TypeInfo(type));
                }
            }
        }
        return formats;
    }
    function get_size(format) {
        var size = 0;
        for (var i in format) {
            size += type_size[format[i].type];
        }
        return size;
    }
    function guess_type(value) {
        if (Number(value)) {
            return Number;
        }
        if (value.match(/^[0-9]{8}T[0-9]{6}([.][0-9]{6})?$/)) {
            return Date;
        }
    }
    Csv.prototype.init = function (data) {
        this.fields = this.options.fields.split(',');
        this.binary = parse(this.options.binary, true);
        this.format = parse(this.options.format);
        this.index = 0;
        if (this.binary) {
            this.dataview = new DataView(data);
            this.dataview.getInt64 = function(byteOffset, littleEndian) {
                var high = littleEndian ? byteOffset + 4 : byteOffset;
                var low = littleEndian ? byteOffset : byteOffset + 4;
                return this.getInt32(high, littleEndian) * Math.pow(2, 32) + this.getUint32(low, littleEndian);
            }
            this.dataview.getUint64 = function(byteOffset, littleEndian) {
                var high = littleEndian ? byteOffset + 4 : byteOffset;
                var low = littleEndian ? byteOffset : byteOffset + 4;
                return this.getUint32(high, littleEndian) * Math.pow(2, 32) + this.getUint32(low, littleEndian);
            }
            this.dataview.getTimestamp = function(byteOffset, littleEndian) {
                var microseconds = this.getInt64(byteOffset, littleEndian);
                var date = new Date(microseconds / 1000);
                date.microseconds = microseconds;
                return date;
            }
            this.dataview.getString = function(byteOffset, length) {
                var s = new Array(length);
                for (var i = 0; i < length; ++i) {
                    s[i] = String.fromCharCode(this.getUint8(byteOffset + i));
                }
                return s.join('');
            }
            this.little_endian = true;
            this.size = get_size(this.binary);
            this.count = this.dataview.byteLength / this.size;
        } else {
            this.data = data.split('\n');
            var last = this.data.pop();
            if (last) {
                this.data.push(last);
            }
            this.count = this.data.length;
        }
    }
    Csv.prototype.readArray = function () {
        if (this.binary) {
            if (this.index >= this.dataview.byteLength) {
                return;
            }
            var array = [];
            for (var i in this.binary) {
                var type_info = this.binary[i];
                var value;
                if (this.index < this.dataview.byteLength && this.fields[i]) {
                    switch (type_info.type) {
                        case 'b' : value = this.dataview.getInt8(this.index);                           break;
                        case 'ub': value = this.dataview.getUint8(this.index);                          break;
                        case 'w' : value = this.dataview.getInt16(this.index, this.little_endian);      break;
                        case 'uw': value = this.dataview.getUint16(this.index, this.little_endian);     break;
                        case 'i' : value = this.dataview.getInt32(this.index, this.little_endian);      break;
                        case 'ui': value = this.dataview.getUint32(this.index, this.little_endian);     break;
                        case 'l' : value = this.dataview.getInt64(this.index, this.little_endian);      break;
                        case 'ul': value = this.dataview.getUint64(this.index, this.little_endian);     break;
                        case 'c':  value = this.dataview.getString(this.index, 1);                      break;
                        case 's':  value = this.dataview.getString(this.index, type_info.length);       break;
                        case 'f' : value = this.dataview.getFloat32(this.index, this.little_endian);    break;
                        case 'd' : value = this.dataview.getFloat64(this.index, this.little_endian);    break;
                        case 't' : value = this.dataview.getTimestamp(this.index, this.little_endian);  break;
                        case 'lt': /* TODO */                                                           break;
                    }
                }
                array.push(value);
                this.index += type_info.type == 's' ? type_info.length : type_size[type_info.type];
            }
            return array;
        }
        if (this.index >= this.data.length) {
            return;
        }
        var array = [];
        var line = this.data[this.index++];
        if (line) {
            array = line.split(',').map(function(currentValue, index, array) {
                if (this.fields[index]) {
                    return currentValue;
                }
            }, this);
            if (!this.format && this.options.guess == 'true') {
                this.format = new Array(array.length);
                array.forEach(function(currentValue, index, array) {
                    if (currentValue) {
                        this.format[index] = new TypeInfo(guess_type(currentValue));
                    }
                }, this);
            }
            if (this.format) {
                array.forEach(function(currentValue, index, array) {
                    if (currentValue == undefined) {
                        return;
                    }
                    var type_info = this.format[index];
                    if (!type_info) {
                        return;
                    }
                    switch (type_info.type) {
                        case 'b':
                        case 'ub':
                        case 'w':
                        case 'uw':
                        case 'i':
                        case 'ui':
                        case 'f':
                        case 'd':
                        case Number:
                            array[index] = Number(currentValue);
                            break;
                        case 't': 
                        case Date:
                            array[index] = parse_date(currentValue);
                            break;
                    }
                }, this);
            }
        }
        return array;
    }
    function assign(object, field, value) {
        var keys = field.split('/');
        var last_index = keys.length - 1;
        for (var j = 0; j < last_index; ++j) {
            var key = keys[j];
            if (!(key in object)) {
                object[key] = {};
            }
            object = object[key];
        }
        field = keys[last_index];
        object[field] = value;
    }
    Csv.prototype.readObject = function() {
        var array = this.readArray();
        if (!array) {
            return;
        }
        var object = {};
        for (var i in array) {
            var field = this.fields[i];
            var value = array[i];
            if (!field) {
                continue;
            }
            assign(object, field, value);
        }
        return object;
    }
    return Csv;
});
