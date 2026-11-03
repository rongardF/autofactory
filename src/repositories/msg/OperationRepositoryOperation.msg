# Copyright (c) 2026, Autofactory
# All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
# 3. Neither the name of the copyright holder nor the names of its contributors
#    may be used to endorse or promote products derived from this software
#    without specific prior written permission.
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS WARRANTIES ARE DISCLAIMED.

string OPERATION_TYPE_DISPENSING="dispensing"

string id
string operation_type
string operation_number
string description
string start_assembly_model_id
string end_assembly_model_id
repositories/OperationRepositoryMaterial material
repositories/OperationRepositoryEndTool end_tool
repositories/OperationRepositoryArg[] operation_args
repositories/OperationRepositoryDispensingPattern[] dispensing_patterns
