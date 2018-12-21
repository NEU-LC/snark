// This file is part of snark, a generic and flexible library for robotics research
// Copyright (c) 2011 The University of Sydney
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// 1. Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
// 3. Neither the name of the University of Sydney nor the
//    names of its contributors may be used to endorse or promote products
//    derived from this software without specific prior written permission.
//
// NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE
// GRANTED BY THIS LICENSE.  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT
// HOLDERS AND CONTRIBUTORS \"AS IS\" AND ANY EXPRESS OR IMPLIED
// WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
// MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
// BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
// WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
// OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
// IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

define(function () {

    function GridOptions(options) {
        function AxisOptions(options, defaults) {
            Object.assign(this, options);
            this.min = this.min || defaults.min || 0;
            this.max = this.max || defaults.max || 10;
            this.step = this.step || defaults.step || 1;
            this.color = this.color || defaults.color || 'rgba(50, 75, 150, 0.9)';
        }

        function GridLineOptions(options) {
            Object.assign(this, options);
            this.show = 'show' in this ? this.show : true;
            this.color = this.color || 'rgba(100, 100, 100, 0.4)';
            this.width = this.width || 1;
        }

        function LabelOptions(options) {
            Object.assign(this, options);
            this.show = this.show || false;
            this.color = this.color || 'rgba(0, 0, 0, 0.9)';
            this.font = this.font || '18px sans-serif';
        }

        Object.assign(this, options);
        this.show = typeof options !== 'undefined';
        this.x = new AxisOptions(this.x, { color: 'rgba(255, 0, 0, 0.5)' });
        this.y = new AxisOptions(this.y, { color: 'rgba(0, 255, 0, 0.5)' });
        this.axis_width = this.axis_width || 2;
        this.step_length = this.step_length || 5;
        this.grid_lines = new GridLineOptions(this.grid_lines);
        this.x_offset = this.x_offset || 0;
        this.y_offset = this.y_offset || 0;
        this.labels = new LabelOptions(this.labels);
    }

    return GridOptions;

});
