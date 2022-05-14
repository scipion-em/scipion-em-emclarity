# **************************************************************************
# *
# * Authors:     Federico P. de Isidro Gomez (fp.deisidro@cnb.csic.es) [1]
# *
# * [1] Centro Nacional de Biotecnologia, CSIC, Spain
# *
# * This program is free software; you can redistribute it and/or modify
# * it under the terms of the GNU General Public License as published by
# * the Free Software Foundation; either version 2 of the License, or
# * (at your option) any later version.
# *
# * This program is distributed in the hope that it will be useful,
# * but WITHOUT ANY WARRANTY; without even the implied warranty of
# * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# * GNU General Public License for more details.
# *
# * You should have received a copy of the GNU General Public License
# * along with this program; if not, write to the Free Software
# * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
# * 02111-1307  USA
# *
# *  All comments concerning this program package may be sent to the
# *  e-mail address 'scipion@cnb.csic.es'
# *
# **************************************************************************

from pyworkflow.tests import *
from pyworkflow.utils import path

from pwem.emlib.image import ImageHandler

import tomo

from emclarity.protocols import ProtEmclarityAutoAlign



class TestEmClarityBase(BaseTest):
    @classmethod
    def setUpClass(cls):
        setupTestProject(cls)

        cls.inputDataSet = DataSet.getDataSet('emclarity_data')
        # posible error en linea siguiente
        cls.inputSoTS = cls.inputDataSet.getFile('ts1')
        # Añadir otro con protEmclarityAutoALign
        cls.protImportTS = cls._runImportTiltSeries(filesPath=os.path.split(cls.inputSoTS)[0],
                                                    pattern="tilt{TS}.st",
                                                    anglesFrom=0,
                                                    voltage=300,
                                                    magnification=105000,
                                                    sphericalAberration=2.7,
                                                    amplitudeContrast=0.1,
                                                    samplingRate=20.2,
                                                    doseInitial=0,
                                                    dosePerFrame=0.3,
                                                    minAngle=-55,
                                                    maxAngle=65.0,
                                                    stepAngle=2.0)
        
        cls.protEmclarityAutoAlign = cls._runAutoAlign(inputSoTS=os.path.split(cls.inputSoTS)[0],
                                                    beadDiameter=10e-0,
                                                    maxResolution=18,
                                                    minSamplingRate=10,
                                                    maxSamplingRate=3,
                                                    iterationsPerBin=3.0,
                                                    nItersNoRotation=3.0,
                                                    patchSizeFactor=4.0,
                                                    patchTrackingBorder=64.0,
                                                    patchOverlap=0.5,
                                                    maxShiftInAngstroms=40.0,
                                                    maxShiftFactor=1.0,
                                                    refineOnBeads=False)

    @classmethod
    def _runImportTiltSeries(cls, filesPath, pattern, voltage, magnification, sphericalAberration, amplitudeContrast,
                             samplingRate, doseInitial, dosePerFrame, anglesFrom=0, minAngle=0.0, maxAngle=0.0,
                             stepAngle=1.0, tiltAxisAngle=-12.5):
        cls.protImportTS = cls.newProtocol(tomo.protocols.ProtImportTs,
                                           filesPath=filesPath,
                                           filesPattern=pattern,
                                           voltage=voltage,
                                           anglesFrom=anglesFrom,
                                           magnification=magnification,
                                           sphericalAberration=sphericalAberration,
                                           amplitudeContrast=amplitudeContrast,
                                           samplingRate=samplingRate,
                                           doseInitial=doseInitial,
                                           dosePerFrame=dosePerFrame,
                                           minAngle=minAngle,
                                           maxAngle=maxAngle,
                                           stepAngle=stepAngle,
                                           tiltAxisAngle=tiltAxisAngle)
        cls.launchProtocol(cls.protImportTS)
        return cls.protImportTS


    @classmethod
    def _runAutoAlign(cls, inputSoTS, beadDiameter, maxResolution, minSamplingRate, maxSamplingRate, iterationsPerBin,
                      nItersNoRotation, patchSizeFactor, patchTrackingBorder, patchOverlap, maxShiftInAngstroms,
                      maxShiftFactor, refineOnBeads):
        # He modificado protocols.conf
        cls.protEmclarityAutoAlign = cls.newProtocol(ProtEmclarityAutoAlign,
                                            inputSoTS=inputSoTS,
                                            beadDiameter=beadDiameter,
                                            maxResolution=maxResolution,
                                            minSamplingRate=minSamplingRate,
                                            maxSamplingRate=maxSamplingRate,
                                            iterationsPerBin=iterationsPerBin,
                                            nItersNoRotation=nItersNoRotation,
                                            patchSizeFactor=patchSizeFactor,
                                            patchTrackingBorder=patchTrackingBorder,
                                            patchOverlap=patchOverlap,
                                            maxShiftInAngstroms=maxShiftInAngstroms,
                                            maxShiftFactor=maxShiftFactor,
                                            refineOnBeads=refineOnBeads)
        cls.launchProtocol(cls.protEmclarityAutoAlign)
        return cls.protEmclarityAutoAlign

    def test_importTSOutput(self):
        self.assertIsNotNone(self.protImportTS.outputTiltSeries)
    def test_emClarityAutoAlign(self):
        self.assertIsNotNone(self.protEmclarityAutoAlign.outputAlignTiltSeries)