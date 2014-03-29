% Generate test datasets for matio library
%
% Copyright (C) 2010   Christopher C. Hulbert
%
% This library is free software; you can redistribute it and/or
% modify it under the terms of the GNU Lesser General Public
% License as published by the Free Software Foundation; either
% version 2.1 of the License, or (at your option) any later version.
%
% This library is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
% Lesser General Public License for more details.
%
% You should have received a copy of the GNU Lesser General Public
% License along with this library; if not, write to the Free Software
% Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

[c,m,e]=computer;

var1 = reshape(1:20,4,5);
var2 = reshape(single(1:20),4,5);
var3 = reshape(int64(1:20),4,5);
var4 = reshape(uint64(1:20),4,5);
var5 = reshape(int32(1:20),4,5);
var6 = reshape(uint32(1:20),4,5);
var7 = reshape(int16(1:20),4,5);
var8 = reshape(uint16(1:20),4,5);
var9 = reshape(int8(1:20),4,5);
var10 = reshape(uint8(1:20),4,5);
var11 = reshape(complex(1:20,21:40),4,5);
var12 = reshape(single(complex(1:20,21:40)),4,5);
var13 = reshape(int64(complex(1:20,21:40)),4,5);
var14 = reshape(uint64(complex(1:20,21:40)),4,5);
var15 = reshape(int32(complex(1:20,21:40)),4,5);
var16 = reshape(uint32(complex(1:20,21:40)),4,5);
var17 = reshape(int16(complex(1:20,21:40)),4,5);
var18 = reshape(uint16(complex(1:20,21:40)),4,5);
var19 = reshape(int8(complex(1:20,21:40)),4,5);
var20 = reshape(uint8(complex(1:20,21:40)),4,5);
var21 = sparse(diag(1:5));
var22 = sparse(diag(complex(1:5,6:10)));
var23 = [];
var24 = ['abcdefghijklmnopqrstuvwxyz';
         'ABCDEFGHIJKLMNOPQRSTUVWXYZ';
         '1234567890!@#$%^&*()-_=+`~';
         '[{]}\|;:''",<.>/?          '];

%% Structure Variables
var25 = struct();
var26 = repmat(struct('field1',[],'field2',[]),0,1);
var27(1).field1 = zeros(0,1);
var27(1).field2 = repmat(' ',0,1);
var27(2).field1 = repmat(struct,0,1);
var27(2).field2 = repmat({zeros(0,0)},0,1);
var28 = [struct('field1',1,'field2',reshape(2:13,3,4));
         struct('field1',14,'field2',reshape(15:26,3,4))];
var29 = [struct('field1',single(1),'field2',reshape(single(2:13),3,4));
         struct('field1',single(14),'field2',reshape(single(15:26),3,4))];
var30 = [struct('field1',int64(1),'field2',reshape(int64(2:13),3,4));
         struct('field1',int64(14),'field2',reshape(int64(15:26),3,4))];
var31 = [struct('field1',uint64(1),'field2',reshape(uint64(2:13),3,4));
         struct('field1',uint64(14),'field2',reshape(uint64(15:26),3,4))];
var32 = [struct('field1',int32(1),'field2',reshape(int32(2:13),3,4));
         struct('field1',int32(14),'field2',reshape(int32(15:26),3,4))];
var33 = [struct('field1',uint32(1),'field2',reshape(uint32(2:13),3,4));
         struct('field1',uint32(14),'field2',reshape(uint32(15:26),3,4))];
var34 = [struct('field1',int16(1),'field2',reshape(int16(2:13),3,4));
         struct('field1',int16(14),'field2',reshape(int16(15:26),3,4))];
var35 = [struct('field1',uint16(1),'field2',reshape(uint16(2:13),3,4));
         struct('field1',uint16(14),'field2',reshape(uint16(15:26),3,4))];
var36 = [struct('field1',int8(1),'field2',reshape(int8(2:13),3,4));
         struct('field1',int8(14),'field2',reshape(int8(15:26),3,4))];
var37 = [struct('field1',uint8(1),'field2',reshape(uint8(2:13),3,4));
         struct('field1',uint8(14),'field2',reshape(uint8(15:26),3,4))];
var38 = [struct('field1',1+51*j,'field2',reshape((2:13)+(52:63)*j,3,4));
         struct('field1',14+64*j,'field2',reshape((15:26)+(65:76)*j,3,4))];
var39 = [struct('field1',single(1+51*j),...
                'field2',reshape(single((2:13)+(52:63)*j),3,4));
         struct('field1',single(14+64*j),...
                'field2',reshape(single((15:26)+(65:76)*j),3,4))];
var40 = [struct('field1',int64(1+51*j),...
                'field2',reshape(int64((2:13)+(52:63)*j),3,4));
         struct('field1',int64(14+64*j),...
                'field2',reshape(int64((15:26)+(65:76)*j),3,4))];
var41 = [struct('field1',uint64(1+51*j),...
                'field2',reshape(uint64((2:13)+(52:63)*j),3,4));
         struct('field1',uint64(14+64*j),...
                'field2',reshape(uint64((15:26)+(65:76)*j),3,4))];
var42 = [struct('field1',int32(1+51*j),...
                'field2',reshape(int32((2:13)+(52:63)*j),3,4));
         struct('field1',int32(14+64*j),...
                'field2',reshape(int32((15:26)+(65:76)*j),3,4))];
var43 = [struct('field1',uint32(1+51*j),...
                'field2',reshape(uint32((2:13)+(52:63)*j),3,4));
         struct('field1',uint32(14+64*j),...
                'field2',reshape(uint32((15:26)+(65:76)*j),3,4))];
var44 = [struct('field1',int16(1+51*j),...
                'field2',reshape(int16((2:13)+(52:63)*j),3,4));
         struct('field1',int16(14+64*j),...
                'field2',reshape(int16((15:26)+(65:76)*j),3,4))];
var45 = [struct('field1',uint16(1+51*j),...
                'field2',reshape(uint16((2:13)+(52:63)*j),3,4));
         struct('field1',uint16(14+64*j),...
                'field2',reshape(uint16((15:26)+(65:76)*j),3,4))];
var46 = [struct('field1',int8(1+51*j),...
                'field2',reshape(int8((2:13)+(52:63)*j),3,4));
         struct('field1',int8(14+64*j),...
                'field2',reshape(int8((15:26)+(65:76)*j),3,4))];
var47 = [struct('field1',uint8(1+51*j),...
                'field2',reshape(uint8((2:13)+(52:63)*j),3,4));
         struct('field1',uint8(14+64*j),...
                'field2',reshape(uint8((15:26)+(65:76)*j),3,4))];

if e == 'B'
    save('-v6',['matio_test_cases_uncompressed_be.mat'],'var*');
    save(['matio_test_cases_compressed_be.mat'],'var*');
    save('-v7.3',['matio_test_cases_hdf_be.mat'],'var*');
else
    save('-v6',['matio_test_cases_uncompressed_le.mat'],'var*');
    save(['matio_test_cases_compressed_le.mat'],'var*');
    save('-v7.3',['matio_test_cases_hdf_le.mat'],'var*');
    save('-v4','matio_test_cases_v4_le.mat','var1','var11','var21');
end
