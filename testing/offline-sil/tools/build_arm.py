#!/usr/bin/env python3
"""Compile public firmware + external STM32CubeF4 Drivers. NEVER flash hardware.

Flags and HAL translation-unit set mirror the inspected CubeIDE Debug build.
This helper is a portable build reproduction, not the old ELF audit implementation.
"""
from pathlib import Path
import argparse,subprocess,hashlib,json,sys
ROOT=Path(__file__).resolve().parents[3]
def main():
 p=argparse.ArgumentParser(description=__doc__)
 p.add_argument('--gcc',type=Path,required=True)
 p.add_argument('--drivers',type=Path,required=True,help='STM32Cube FW_F4 V1.28.3 Drivers directory')
 p.add_argument('--out',type=Path,required=True)
 a=p.parse_args();out=a.out.resolve();proj=ROOT/'firmware/stm32/LKM_BNO086_Bringup'
 if ROOT/'.local' not in out.parents:raise SystemExit('Output must be a fresh directory under repository .local')
 if out.exists():raise SystemExit('Refusing to overwrite build directory')
 out.mkdir(parents=True);log=[]
 def run(cmd):
  r=subprocess.run([str(x) for x in cmd],stdout=subprocess.PIPE,stderr=subprocess.STDOUT,check=False)
  text=r.stdout.decode('utf-8',errors='replace');log.append(text)
  if r.returncode:print(text);raise RuntimeError('Compiler command failed')
  return text
 common=['-mcpu=cortex-m4','-mfpu=fpv4-sp-d16','-mfloat-abi=hard','-mthumb']
 flags=common+['-std=gnu11','-g3','-DDEBUG','-DUSE_HAL_DRIVER','-DSTM32F405xx','-O0','-ffunction-sections','-fdata-sections','-Wall','--specs=nano.specs']
 inc=[proj/'Core/Inc',a.drivers/'STM32F4xx_HAL_Driver/Inc',a.drivers/'STM32F4xx_HAL_Driver/Inc/Legacy',a.drivers/'CMSIS/Device/ST/STM32F4xx/Include',a.drivers/'CMSIS/Include']
 for d in inc:
  if not d.is_dir():raise SystemExit('Missing dependency '+str(d))
  flags+=['-I',str(d)]
 hal=['','_cortex','_dma','_dma_ex','_exti','_flash','_flash_ex','_flash_ramfunc','_gpio','_pwr','_pwr_ex','_rcc','_rcc_ex','_spi','_uart']
 sources=sorted((proj/'Core/Src').glob('*.c'))+[a.drivers/'STM32F4xx_HAL_Driver/Src'/('stm32f4xx_hal'+s+'.c') for s in hal]
 compiler=run([a.gcc,'--version']).splitlines()[0];objects=[]
 try:
  for src in sources:
   obj=out/(src.stem+'.o');run([a.gcc,*flags,'-c',src,'-o',obj]);objects.append(obj)
  obj=out/'startup.o';run([a.gcc,*common,'-x','assembler-with-cpp','-c',proj/'Core/Startup/startup_stm32f405rgtx.s','-o',obj]);objects.append(obj)
  elf=out/'LKM_BNO086_Bringup.elf'
  run([a.gcc,*objects,*common,'-T',proj/'STM32F405RGTX_FLASH.ld','--specs=nosys.specs','--specs=nano.specs','-Wl,--gc-sections','-static','-Wl,--start-group','-lc','-lm','-Wl,--end-group','-o',elf])
  b=elf.read_bytes();assert b[:6]==b'\x7fELF\x01\x01' and int.from_bytes(b[18:20],'little')==40
  nm=a.gcc.with_name(a.gcc.name.replace('gcc','nm'));symbols=run([nm,'--defined-only',elf])
  required=['main','LKM_BNO086_Diag_Service','LKM_DualIMU_Diag_Service','LKM_FC100Hz_Bridge_Service','LKM_AW_Shadow_Service','LKM_RP_FrameMonitor_Service','LKM_FC_ActuatorMonitor_Service','MX_SPI1_Init','MX_SPI2_Init','MX_USART6_UART_Init']
  missing=[s for s in required if not any(line.endswith(' '+s) for line in symbols.splitlines())];assert not missing,missing
  # Generated functions may be link-GC'd by production hard locks; verify in objects, not require dead code in ELF.
  for mod,symbol in [('LKM_AW_Core_F405','LKM_AW_Core_F405_step'),('LKM_AW_Ctrl_F405','LKM_AW_Ctrl_F405'),('LKM_AW_Mixer_F405','LKM_AW_Mixer_F405')]:
   s=run([nm,'--defined-only',out/(mod+'.o')]);assert any(line.endswith(' '+symbol) for line in s.splitlines()),symbol
  result={'scope':'ARM_COMPILE_LINK_ONLY_NOT_MCU_EXECUTION','compiler':compiler,'translation_units':len(objects),'elf_sha256':hashlib.sha256(b).hexdigest(),'required_linked_symbols':required,'status':'PASS'}
  (out/'result.json').write_text(json.dumps(result,indent=2),encoding='utf-8');print(json.dumps(result,indent=2));return 0
 finally:(out/'build.txt').write_text('\n'.join(log),encoding='utf-8')
if __name__=='__main__':sys.exit(main())
