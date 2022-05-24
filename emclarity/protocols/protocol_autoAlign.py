# -*- coding: utf-8 -*-
# **************************************************************************
# *
# * Authors:     you (you@yourinstitution.email)
# *
# * your institution
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
# *  e-mail address 'you@yourinstitution.email'
# *
# **************************************************************************


"""
Describe your python module here:
This module will provide the traditional Hello world example
"""
import os

import numpy as np

import pwem.objects as data
from pwem.objects import Transform

from pyworkflow import BETA
from pyworkflow.object import Set, Integer
import pyworkflow.utils.path as path
import pyworkflow.protocol.params as params
from pwem.protocols import EMProtocol

import tomo.objects as tomoObj
from tomo.protocols import ProtTomoBase
from tomo.objects import SetOfTiltSeries

from emclarity import Plugin
import imod.utils as utils
from imod import Plugin as imodplugin

PARAMS_FN = 'param.m'
RAWTLT_FN = 'tilt.rawtlt'
EMCLARITY_STACK_FOLDER = 'fixedStacks'
EXT_XF = '.xf'
EXT_3DFIND_ALI = '_3dfind.ali'
EXT_FIXED = '.fixed'


class ProtEmclarityAutoAlign(EMProtocol, ProtTomoBase):
    """
    This protocol will print hello world in the console
    IMPORTANT: Classes names should be unique, better prefix them
    """
    _label = 'autoAlign'

    # -------------------------- DEFINE param functions ----------------------
    def _defineParams(self, form):
        """ Define the input parameters that will be used.
        Params:
            form: this is the form to be populated with sections and params.
        """
        # You need a params to belong to a section:
        form.addSection('Input')

        form.addParam('inputSetOfTiltSeries',
                      params.PointerParam,
                      pointerClass='SetOfTiltSeries',
                      important=True,
                      label='Input set of tilt-series.')

        form.addParam('beadDiameter', params.FloatParam,
                      default=5,
                      important=True,
                      label='Bead diameter (nm)',
                      help='Bead diameter in meters (e.g. 10e-9). If 0, the beads are ignored during the alignment')

        form.addParam('tiltAxis', params.FloatParam,
                      allowsNull=True,
                      expertLevel=params.LEVEL_ADVANCED,
                      label='Tilt axis (degrees)',
                      help='Tilt axis orientation in degrees.')

        form.addSection('expert params')
        form.addParam('autoAli_max_resolution', params.FloatParam,
                      allowsNull=True,
                      label='Max Resolution (Å)',
                      expertLevel=params.LEVEL_ADVANCED,
                      help='Low-pass cutoff, in Å, used in alignment. A suggested value is 18 Å')

        form.addParam('autoAli_min_sampling_rate', params.FloatParam,
                      allowsNull=True,
                      label='Min sampling rate (Å/px)',
                      expertLevel=params.LEVEL_ADVANCED,
                      help='Maximum pixel size used for alignment, in Å per pixel. A suggested value is 10 Å ')

        form.addParam('autoAli_max_sampling_rate', params.FloatParam,
                      allowsNull=True,
                      label='Max sampling rate (Å/px)',
                      expertLevel=params.LEVEL_ADVANCED,
                      help='Minimum pixel size used for alignment, in Å per pixel. A suggested value is 3 Å')

        form.addParam('autoAli_iterations_per_bin', params.FloatParam,
                      allowsNull=True,
                      label='Iterations per bin',
                      expertLevel=params.LEVEL_ADVANCED,
                      help='The number of patch tracking iterations, for each bin. A suggested value is 3 iterations')

        form.addParam('autoAli_n_iters_no_rotation', params.FloatParam,
                      allowsNull=True,
                      label='n iters no rotation',
                      expertLevel=params.LEVEL_ADVANCED,
                      help='The number of patch tracking iterations, for each bin,'
                           'before activating local alignments. A suggested value is 3 iterations')

        form.addParam('autoAli_patch_size_factor', params.FloatParam,
                      allowsNull=True,
                      label='Patch size factor',
                      expertLevel=params.LEVEL_ADVANCED,
                      help='Sets the size of the patches used for patch tracking.'
                           'Making this larger will result in more patches, and more local areas in later iterations,'
                           'but may also decrease accuracy. A suggested Patch size factor is 4')

        form.addParam('autoAli_patch_tracking_border', params.FloatParam,
                      allowsNull=True,
                      label='Patch tracking border',
                      expertLevel=params.LEVEL_ADVANCED,
                      help='Number of pixels to trim off each edge in X and in Y. A suggested value is 64')

        form.addParam('autoAli_patch_overlap', params.FloatParam,
                      allowsNull=True,
                      label='Patch overlap',
                      expertLevel=params.LEVEL_ADVANCED,
                      help='Fractional overlap in X and Y between patches that are tracked by correlation.'
                           'This influences the number of patches. A suggested value is 0.5')

        form.addParam('autoAli_max_shift_in_angstroms', params.FloatParam,
                      allowsNull=True,
                      label='Max shift (Å)',
                      expertLevel=params.LEVEL_ADVANCED,
                      help='Maximum shifts allowed, in Å, for the patch tracking alignment. Suggested 40 Å')

        form.addParam('autoAli_max_shift_factor', params.FloatParam,
                      allowsNull=True,
                      label='Max shift factor',
                      expertLevel=params.LEVEL_ADVANCED,
                      help='The maximum shifts allowed are progressively reduced with the iterations i. A suggested factor is 1.0')

        form.addParam('autoAli_refine_on_beads', params.BooleanParam,
                      default=False,
                      label='Refine on beads',
                      expertLevel=params.LEVEL_ADVANCED,
                      help='Whether or not the patch tracking alignment should be refined using the gold beads.'
                           'This refinement makes the alignment significantly slower, but can substantially '
                           'improve the quality of the alignment. Suggested as False')

    # --------------------------- STEPS functions ------------------------------
    def _insertAllSteps(self):
        for ts in self.inputSetOfTiltSeries.get():
            self._insertFunctionStep(self.autoAlignStep, ts.getObjId())
        self._insertFunctionStep(self.closeOutputSetsStep)

    def autoAlignStep(self, tsObjId):
        """Compute the alignment of the tilt series"""

        ts = self.inputSetOfTiltSeries.get()[tsObjId]
        tsId = ts.getTsId()
        workingDir = self._getExtraPath(tsId)

        path.makePath(workingDir)

        stack = os.path.abspath(ts.getFirstItem().getFileName())
        rawtlt = os.path.join(workingDir, RAWTLT_FN)
        ts.generateTltFile(rawtlt)

        if self.tiltAxis.hasValue():
            rotationAngle = self.tiltAxis.get()
        else:
            rotationAngle = ts.getAcquisition().getTiltAxisAngle()

        sampling = ts.getSamplingRate()
        self.create_parameters_file(sampling, workingDir)

        argsAutoAlign = "%s" % PARAMS_FN
        argsAutoAlign += " %s" % stack
        argsAutoAlign += " %s" % RAWTLT_FN
        argsAutoAlign += " %s" % rotationAngle

        Plugin.runEmClarity(self, 'autoAlign', argsAutoAlign, cwd=self._getExtraPath(tsId))

        xfFile = os.path.join(workingDir, EMCLARITY_STACK_FOLDER, tsId + EXT_XF)
        alifile = os.path.join(workingDir, EMCLARITY_STACK_FOLDER, tsId + EXT_3DFIND_ALI)
        stackfixed = os.path.join(workingDir, EMCLARITY_STACK_FOLDER, tsId + EXT_FIXED)
        argsImod = ' -xform %s %s %s' % (xfFile, stackfixed, alifile)

        imodplugin.runImod(self, 'newstack', argsImod)

        self.defineOutputObjects(self.inputSetOfTiltSeries.get())

        alignmentMatrix = utils.formatTransformationMatrix(xfFile)

        newTs = tomoObj.TiltSeries(tsId=tsId)
        newTs.copyInfo(ts)

        self.outputSetOfTiltSeries.append(newTs)

        for index, tiltImage in enumerate(ts):
            newTi = tomoObj.TiltImage()
            newTi.copyInfo(tiltImage, copyId=True, copyTM=False)

            if tiltImage.hasTransform():
                transform = Transform()
                previousTransform = tiltImage.getTransform().getMatrix()
                newTransform = alignmentMatrix[:, :, index]
                previousTransformArray = np.array(previousTransform)
                newTransformArray = np.array(newTransform)
                outputTransformMatrix = np.matmul(newTransformArray, previousTransformArray)
                transform.setMatrix(outputTransformMatrix)
                newTi.setTransform(transform)

            else:
                transform = Transform()
                newTransform = alignmentMatrix[:, :, index]
                newTransformArray = np.array(newTransform)
                transform.setMatrix(newTransformArray)
                newTi.setTransform(transform)

            newTi.setAcquisition(tiltImage.getAcquisition())

            newTi.setLocation(index + 1, alifile)

            newTs.append(newTi)

        newTs.write(properties=False)

        self.outputSetOfTiltSeries.update(newTs)
        self.outputSetOfTiltSeries.write()

        self._store()


    def defineOutputObjects(self, inputSet):
        """ Method to generate output classes of set of tilt-series"""

        if hasattr(self, "outputSetOfTiltSeries"):
            self.outputSetOfTiltSeries.enableAppend()

        else:
            outputSetOfTiltSeries = self._createSetOfTiltSeries(suffix='aligned')

            if isinstance(inputSet, SetOfTiltSeries):
                outputSetOfTiltSeries.copyInfo(inputSet)
                outputSetOfTiltSeries.setDim(inputSet.getDim())

            elif isinstance(inputSet, SetOfTomograms):
                outputSetOfTiltSeries.setAcquisition(inputSet.getAcquisition())
                outputSetOfTiltSeries.setSamplingRate(inputSet.getSamplingRate())
                outputSetOfTiltSeries.setDim(inputSet.getDim())

            outputSetOfTiltSeries.setStreamState(Set.STREAM_OPEN)

            self._defineOutputs(outputSetOfTiltSeries=outputSetOfTiltSeries)
            self._defineSourceRelation(inputSet, outputSetOfTiltSeries)

        return self.outputSetOfTiltSeries

    def closeOutputSetsStep(self):
        self.outputSetOfTiltSeries.setStreamState(Set.STREAM_CLOSED)
        self.outputSetOfTiltSeries.write()
        self._store()

    def create_parameters_file(self, sampling, pathtoTS):

        f = open(os.path.join(pathtoTS, PARAMS_FN), 'w')

        pixel_size = sampling * 1e-10
        f.write('nGPUs=1')
        f.write('\nnCpuCores=1')
        f.write('\nPIXEL_SIZE=' + str(pixel_size))
        f.write('\nbeadDiameter=' + str(float(self.beadDiameter) * 1e-9))
        if self.autoAli_max_resolution.hasValue():
            f.write('\nautoAli_max_resolution=' + str(float(self.autoAli_max_resolution)))
        if self.autoAli_min_sampling_rate.hasValue():
            f.write('\nautoAli_min_sampling_rate=' + str(float(self.autoAli_min_sampling_rate)))
        if self.autoAli_max_sampling_rate.hasValue():
            f.write('\nautoAli_max_sampling_rate=' + str(float(self.autoAli_max_sampling_rate)))
        if self.autoAli_iterations_per_bin.hasValue():
            f.write('\nautoAli_iterations_per_bin=' + str(self.autoAli_iterations_per_bin))
        if self.autoAli_n_iters_no_rotation.hasValue():
            f.write('\nautoAli_n_iters_no_rotation=' + str(self.autoAli_n_iters_no_rotation))
        if self.autoAli_patch_size_factor.hasValue():
            f.write('\nautoAli_patch_size_factor=' + str(self.autoAli_patch_size_factor))
        if self.autoAli_patch_tracking_border.hasValue():
            f.write('\nautoAli_patch_tracking_border=' + str(self.autoAli_patch_tracking_border))
        if self.autoAli_patch_overlap.hasValue():
            f.write('\nautoAli_patch_overlap=' + str(self.autoAli_patch_overlap))
        if self.autoAli_max_shift_in_angstroms.hasValue():
            f.write('\nautoAli_max_shift_in_angstroms=' + str(self.autoAli_max_shift_in_angstroms))
        if self.autoAli_max_shift_factor.hasValue():
            f.write('\nautoAli_max_shift_factor=' + str(self.autoAli_max_shift_factor))
        if self.autoAli_refine_on_beads.hasValue():
            f.write('\nautoAli_refine_on_beads=' + str(self.autoAli_refine_on_beads).lower())
        f.close()


    # --------------------------- INFO functions -----------------------------------
    def _summary(self):
        """ Summarize what the protocol has done"""
        summary = []

        if self.isFinished():
            summary.append("This protocol has printed *%s* %i times." % (self.message, self.times))
        return summary

    def _methods(self):
        methods = []

        if self.isFinished():
            methods.append("%s has been printed in this run %i times." % (self.message, self.times))
            if self.previousCount.hasPointer():
                methods.append("Accumulated count from previous runs were %i."
                               " In total, %s messages has been printed."
                               % (self.previousCount, self.count))
        return methods
