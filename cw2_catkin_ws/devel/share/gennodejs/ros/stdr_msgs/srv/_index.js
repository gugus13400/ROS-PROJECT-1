
"use strict";

let DeleteSoundSource = require('./DeleteSoundSource.js')
let LoadExternalMap = require('./LoadExternalMap.js')
let DeleteCO2Source = require('./DeleteCO2Source.js')
let RegisterGui = require('./RegisterGui.js')
let AddSoundSource = require('./AddSoundSource.js')
let DeleteThermalSource = require('./DeleteThermalSource.js')
let LoadMap = require('./LoadMap.js')
let AddThermalSource = require('./AddThermalSource.js')
let DeleteRfidTag = require('./DeleteRfidTag.js')
let AddCO2Source = require('./AddCO2Source.js')
let MoveRobot = require('./MoveRobot.js')
let AddRfidTag = require('./AddRfidTag.js')

module.exports = {
  DeleteSoundSource: DeleteSoundSource,
  LoadExternalMap: LoadExternalMap,
  DeleteCO2Source: DeleteCO2Source,
  RegisterGui: RegisterGui,
  AddSoundSource: AddSoundSource,
  DeleteThermalSource: DeleteThermalSource,
  LoadMap: LoadMap,
  AddThermalSource: AddThermalSource,
  DeleteRfidTag: DeleteRfidTag,
  AddCO2Source: AddCO2Source,
  MoveRobot: MoveRobot,
  AddRfidTag: AddRfidTag,
};
