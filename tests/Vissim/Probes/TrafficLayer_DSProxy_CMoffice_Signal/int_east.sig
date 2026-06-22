<?xml version="1.0" encoding="UTF-8"?>
<sc id="3" name="" frequency="1" defaultIntergreenMatrix="0">
  <sgs>
    <sg id="1" name="" defaultSignalSequence="3">
      <defaultDurations>
        <defaultDuration display="2" duration="0" />
        <defaultDuration display="4" duration="3000" />
        <defaultDuration display="1" duration="0" />
        <defaultDuration display="3" duration="0" />
      </defaultDurations>
    </sg>
    <sg id="2" name="" defaultSignalSequence="3">
      <defaultDurations>
        <defaultDuration display="2" duration="0" />
        <defaultDuration display="4" duration="3000" />
        <defaultDuration display="1" duration="0" />
        <defaultDuration display="3" duration="0" />
      </defaultDurations>
    </sg>
    <sg id="3" name="" defaultSignalSequence="3">
      <defaultDurations>
        <defaultDuration display="2" duration="0" />
        <defaultDuration display="4" duration="3000" />
        <defaultDuration display="1" duration="0" />
        <defaultDuration display="3" duration="0" />
      </defaultDurations>
    </sg>
  </sgs>
  <intergreenmatrices />
  <progs>
    <prog id="1" cycletime="90000" switchpoint="0" offset="0" intergreens="0" name="Signal program">
      <sgs>
        <sg sg_id="1" signal_sequence="3">
          <cmds>
            <cmd display="3" begin="0" />
            <cmd display="1" begin="50000" />
          </cmds>
          <fixedstates>
            <fixedstate display="2" duration="0" />
            <fixedstate display="4" duration="3000" />
          </fixedstates>
        </sg>
        <sg sg_id="2" signal_sequence="3">
          <cmds>
            <cmd display="3" begin="41000" />
            <cmd display="1" begin="0" />
          </cmds>
          <fixedstates>
            <fixedstate display="2" duration="0" />
            <fixedstate display="4" duration="3000" />
          </fixedstates>
        </sg>
        <sg sg_id="3" signal_sequence="3">
          <cmds>
            <cmd display="3" begin="50000" />
            <cmd display="1" begin="0" />
          </cmds>
          <fixedstates>
            <fixedstate display="2" duration="0" />
            <fixedstate display="4" duration="3000" />
          </fixedstates>
        </sg>
      </sgs>
    </prog>
  </progs>
  <stages />
  <interstageProgs />
  <stageProgs />
  <dailyProgLists />
</sc>
