-------------------------------------------------------------------------------
--
-- DE1 top-level module for the Apple ][
--
-- Michel Stempin, michel.stempin@wanadoo.fr
--
-- Based on DE2 top-level by Stephen A. Edwards, Columbia University, sedwards@cs.columbia.edu
--
-- From an original by Terasic Technology, Inc.
-- (DE2_TOP.v, part of the DE2 system board CD supplied by Altera)
--
-------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity hex7seg is 
  port (
    input  : in  unsigned(3 downto 0);
    output : out unsigned(6 downto 0));
end hex7seg;

architecture combinational of hex7seg is
  signal output_n : unsigned(6 downto 0);
begin
  with input select
    output_n <=
    "0111111" when "0000",
    "0000110" when "0001",
    "1011011" when "0010",
    "1001111" when "0011",
    "1100110" when "0100",
    "1101101" when "0101",
    "1111101" when "0110",
    "0000111" when "0111",
    "1111111" when "1000",
    "1101111" when "1001", 
    "1110111" when "1010",
    "1111100" when "1011", 
    "0111001" when "1100", 
    "1011110" when "1101", 
    "1111001" when "1110",
    "1110001" when "1111",
    "XXXXXXX" when others;

  output <= not output_n;

end combinational;

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity DE1_TOP is

  port (
    -- Clocks
    
    CLOCK_24,                                      -- 24 MHz
    CLOCK_27 : in std_logic_vector(1 downto 0);    -- 27 MHz
    CLOCK_50,                                      -- 50 MHz
    EXT_CLOCK : in std_logic;                      -- External Clock

    -- Buttons and switches
    
    KEY : in std_logic_vector(3 downto 0);         -- Push buttons
    SW  : in unsigned(9 downto 0);                 -- DPDT switches

    -- 7-segment displays

    HEX0, HEX1, HEX2, HEX3                         -- 7-segment displays
        : out unsigned(6 downto 0);

    -- LED displays

    LEDG : out std_logic_vector(7 downto 0);       -- Green LEDs
    LEDR : out std_logic_vector(9 downto 0);       -- Red LEDs

    -- RS-232 interface

    UART_TXD : out std_logic;                      -- UART transmitter   
    UART_RXD : in std_logic;                       -- UART receiver


    -- SDRAM
    
    DRAM_DQ : inout std_logic_vector(15 downto 0); -- Data Bus
    DRAM_ADDR : out std_logic_vector(11 downto 0); -- Address Bus    
    DRAM_LDQM,                                     -- Low-byte Data Mask 
    DRAM_UDQM,                                     -- High-byte Data Mask
    DRAM_WE_N,                                     -- Write Enable
    DRAM_CAS_N,                                    -- Column Address Strobe
    DRAM_RAS_N,                                    -- Row Address Strobe
    DRAM_CS_N,                                     -- Chip Select
    DRAM_BA_0,                                     -- Bank Address 0
    DRAM_BA_1,                                     -- Bank Address 0
    DRAM_CLK,                                      -- Clock
    DRAM_CKE : out std_logic;                      -- Clock Enable

    -- FLASH
    
    FL_DQ : inout std_logic_vector(7 downto 0);      -- Data bus
    FL_ADDR : out std_logic_vector(21 downto 0);  -- Address bus
    FL_WE_N,                                         -- Write Enable
    FL_RST_N,                                        -- Reset
    FL_OE_N,                                         -- Output Enable
    FL_CE_N : out std_logic;                         -- Chip Enable

    -- SRAM
    
    SRAM_DQ : inout unsigned(15 downto 0);         -- Data bus 16 Bits
    SRAM_ADDR : out unsigned(17 downto 0);         -- Address bus 18 Bits
    SRAM_UB_N,                                     -- High-byte Data Mask 
    SRAM_LB_N,                                     -- Low-byte Data Mask 
    SRAM_WE_N,                                     -- Write Enable
    SRAM_CE_N,                                     -- Chip Enable
    SRAM_OE_N : out std_logic;                     -- Output Enable

    -- SD card interface
    
    SD_DAT : in std_logic;      -- SD Card Data      SD pin 7 "DAT 0/DataOut"
    SD_DAT3 : out std_logic;    -- SD Card Data 3    SD pin 1 "DAT 3/nCS"
    SD_CMD : out std_logic;     -- SD Card Command   SD pin 2 "CMD/DataIn"
    SD_CLK : out std_logic;     -- SD Card Clock     SD pin 5 "CLK"

    -- USB JTAG link
    
    TDI,                        -- CPLD -> FPGA (data in)
    TCK,                        -- CPLD -> FPGA (clk)
    TCS : in std_logic;         -- CPLD -> FPGA (CS)
    TDO : out std_logic;        -- FPGA -> CPLD (data out)

    -- I2C bus
    
    I2C_SDAT : inout std_logic; -- I2C Data
    I2C_SCLK : out std_logic;   -- I2C Clock

    -- PS/2 port

    PS2_DAT,                    -- Data
    PS2_CLK : in std_logic;     -- Clock

    -- VGA output
    
    VGA_HS,                                        -- H_SYNC
    VGA_VS : out std_logic;                        -- V_SYNC
    VGA_R,                                         -- Red[3:0]
    VGA_G,                                         -- Green[3:0]
    VGA_B : out unsigned(3 downto 0);              -- Blue[3:0]

    -- Audio CODEC
    
    AUD_ADCLRCK : inout std_logic;                      -- ADC LR Clock
    AUD_ADCDAT : in std_logic;                          -- ADC Data
    AUD_DACLRCK : inout std_logic;                      -- DAC LR Clock
    AUD_DACDAT : out std_logic;                         -- DAC Data
    AUD_BCLK : inout std_logic;                         -- Bit-Stream Clock
    AUD_XCK : out std_logic;                            -- Chip Clock
    
    -- General-purpose I/O
    
    GPIO_0,                                      -- GPIO Connection 0
    GPIO_1 : inout std_logic_vector(35 downto 0) -- GPIO Connection 1   
    );
  
end DE1_TOP;

architecture datapath of DE1_TOP is

  component CLK28MPLL is
    port (
      inclk0    : in std_logic;
      c0        : out std_logic;
      c1        : out std_logic);
  end component;
  
  signal CLK_28M, CLK_14M, CLK_2M, PRE_PHASE_ZERO : std_logic;
  signal IO_SELECT, DEVICE_SELECT : std_logic_vector(7 downto 0);
  signal ADDR : unsigned(15 downto 0);
  signal D, PD : unsigned(7 downto 0);

  signal ram_we : std_logic;
  signal VIDEO, HBL, VBL, LD194 : std_logic;
  signal COLOR_LINE : std_logic;
  signal COLOR_LINE_CONTROL : std_logic;
  signal GAMEPORT : std_logic_vector(7 downto 0);
  signal cpu_pc : unsigned(15 downto 0);

  signal K : unsigned(7 downto 0);
  signal read_key : std_logic;

  signal flash_clk : unsigned(22 downto 0) := (others => '0');
  signal power_on_reset : std_logic := '1';
  signal reset : std_logic;

  signal speaker : std_logic;

  signal track : unsigned(5 downto 0);
  signal image : unsigned(9 downto 0);
  signal trackmsb : unsigned(3 downto 0);
  signal D1_ACTIVE, D2_ACTIVE : std_logic;
  signal track_addr : unsigned(13 downto 0);
  signal TRACK_RAM_ADDR : unsigned(13 downto 0);
  signal tra : unsigned(15 downto 0);
  signal TRACK_RAM_DI : unsigned(7 downto 0);
  signal TRACK_RAM_WE : std_logic;
  signal R_10 : unsigned(9 downto 0);
  signal G_10 : unsigned(9 downto 0);
  signal B_10 : unsigned(9 downto 0);

  signal CS_N, MOSI, MISO, SCLK : std_logic;

begin

  reset <= (not KEY(3)) or power_on_reset;

  power_on : process(CLK_14M)
  begin
    if rising_edge(CLK_14M) then
      if flash_clk(22) = '1' then
        power_on_reset <= '0';
      end if;
    end if;
  end process;

  -- In the Apple ][, this was a 555 timer
  flash_clkgen : process (CLK_14M)
  begin
    if rising_edge(CLK_14M) then
      flash_clk <= flash_clk + 1;
    end if;     
  end process;

  -- Use a PLL to divide the 50 MHz down to 28 MHz and 14 MHz
  pll : CLK28MPLL port map (
    inclk0 => CLOCK_50,
    c0     => CLK_28M,
    c1     => CLK_14M
    );

  -- Paddle buttons
  GAMEPORT <=  "0000" & (not KEY(2 downto 0)) & "0";

  COLOR_LINE_CONTROL <= COLOR_LINE and SW(9);  -- Color or B&W mode
  
  core : entity work.apple2 port map (
    CLK_14M        => CLK_14M,
    CLK_2M         => CLK_2M,
    PRE_PHASE_ZERO => PRE_PHASE_ZERO,
    FLASH_CLK      => flash_clk(22),
    reset          => reset,
    ADDR           => ADDR,
    ram_addr       => SRAM_ADDR(15 downto 0),
    D              => D,
    ram_do         => SRAM_DQ(7 downto 0),
    PD             => PD,
    ram_we         => ram_we,
    VIDEO          => VIDEO,
    COLOR_LINE     => COLOR_LINE,
    HBL            => HBL,
    VBL            => VBL,
    LD194          => LD194,
    K              => K,
    read_key       => read_key,
    AN             => LEDG(7 downto 4),
    GAMEPORT       => GAMEPORT,
    IO_SELECT      => IO_SELECT,
    DEVICE_SELECT  => DEVICE_SELECT,
    pcDebugOut     => cpu_pc,
    speaker        => speaker
    );

  vga : entity work.vga_controller port map (
    CLK_28M    => CLK_28M,
    VIDEO      => VIDEO,
    COLOR_LINE => COLOR_LINE_CONTROL,
    HBL        => HBL,
    VBL        => VBL,
    LD194      => LD194,
    VGA_HS     => VGA_HS,
    VGA_VS     => VGA_VS,
    VGA_R      => R_10,
    VGA_G      => G_10,
    VGA_B      => B_10
    );

  VGA_R <= R_10(9 downto 6);
  VGA_G <= G_10(9 downto 6);
  VGA_B <= B_10(9 downto 6);

  keyboard : entity work.keyboard port map (
    PS2_Clk  => PS2_CLK,
    PS2_Data => PS2_DAT,
    CLK_14M  => CLK_14M,
    reset    => reset,
    read     => read_key,
    K        => K
    );

  disk : entity work.disk_ii port map (
    CLK_14M        => CLK_14M,
    CLK_2M         => CLK_2M,
    PRE_PHASE_ZERO => PRE_PHASE_ZERO,
    IO_SELECT      => IO_SELECT(6),
    DEVICE_SELECT  => DEVICE_SELECT(6),
    RESET          => reset,
    A              => ADDR,
    D_IN           => D,
    D_OUT          => PD,
    TRACK          => TRACK,
    TRACK_ADDR     => TRACK_ADDR,
    D1_ACTIVE      => D1_ACTIVE,
    D2_ACTIVE      => D2_ACTIVE,
    ram_write_addr => TRACK_RAM_ADDR,
    ram_di         => TRACK_RAM_DI,
    ram_we         => TRACK_RAM_WE
    );

  sdcard_interface : entity work.spi_controller port map (
    CLK_14M        => CLK_14M,
    RESET          => RESET,

    CS_N           => CS_N,
    MOSI           => MOSI,
    MISO           => MISO,
    SCLK           => SCLK,
    
    track          => TRACK,
    image          => image,
    
    ram_write_addr => TRACK_RAM_ADDR,
    ram_di         => TRACK_RAM_DI,
    ram_we         => TRACK_RAM_WE
    );

  image <= "0" & SW(8 downto 0);

  SD_DAT3 <= CS_N;
  SD_CMD  <= MOSI;
  MISO    <= SD_DAT;
  SD_CLK  <= SCLK;

  i2c : entity work.i2c_controller port map (
    CLK   => CLOCK_50,
    SCLK  => I2C_SCLK,
    SDAT  => I2C_SDAT,
    reset => reset
  );

  audio_output : entity work.wm8731_audio port map (
    clk          => CLK_14M,
    reset        => reset,
    data         => speaker & "000000000000000",
  
    -- Audio interface signals
    AUD_ADCLRCK  => AUD_ADCLRCK,
    AUD_ADCDAT   => AUD_ADCDAT,
    AUD_DACLRCK  => AUD_DACLRCK,
    AUD_DACDAT   => AUD_DACDAT,
    AUD_BCLK     => AUD_BCLK
  );

  AUD_XCK <= CLK_14M;

  -- Current disk track on right two digits 
  trackmsb <= "00" & track(5 downto 4);
  digit0 : entity work.hex7seg port map (track(3 downto 0), HEX0);
  digit1 : entity work.hex7seg port map (trackmsb, HEX1);

  -- Current disk image on left two digits
  digit2 : entity work.hex7seg port map (image(3 downto 0), HEX2);
  digit3 : entity work.hex7seg port map (image(7 downto 4), HEX3);

  SRAM_DQ(7 downto 0) <= D when ram_we = '1' else (others => 'Z');
  SRAM_ADDR(17) <= '0';
  SRAM_ADDR(16) <= '0';
  SRAM_UB_N <= '1';
  SRAM_LB_N <= '0';
  SRAM_CE_N <= '0';
  SRAM_WE_N <= not ram_we;
  SRAM_OE_N <= ram_we;



  -- Decode the PC on the red LEDs
  LEDR(9) <= cpu_pc(15);
  LEDR(8) <= cpu_pc(14);
  LEDR(7) <= cpu_pc(13);
  LEDR(6) <= cpu_pc(12);
  LEDR(5) <= cpu_pc(11);
  LEDR(4) <= cpu_pc(10);
  LEDR(3) <= cpu_pc(9);
  LEDR(2) <= cpu_pc(8);
  LEDR(1) <= cpu_pc(7);
  LEDR(0) <= cpu_pc(6);

  -- LEDG(7 downto 4) are AN outputs
  LEDG(3) <= '0';
  LEDG(2) <= D2_ACTIVE;
  LEDG(1) <= D1_ACTIVE;
  LEDG(0) <= speaker;
  
  UART_TXD <= '0';
  DRAM_ADDR <= (others => '0');
  DRAM_LDQM <= '0';
  DRAM_UDQM <= '0';
  DRAM_WE_N <= '1';
  DRAM_CAS_N <= '1';
  DRAM_RAS_N <= '1';
  DRAM_CS_N <= '1';
  DRAM_BA_0 <= '0';
  DRAM_BA_1 <= '0';
  DRAM_CLK <= '0';
  DRAM_CKE <= '0';
  FL_ADDR <= (others => '0');
  FL_WE_N <= '1';
  FL_RST_N <= '0';
  FL_OE_N <= '1';
  FL_CE_N <= '1';

  TDO <= '0';

  -- Set all other unused bidirectional ports to tri-state
  DRAM_DQ     <= (others => 'Z');
  FL_DQ       <= (others => 'Z');
  SRAM_DQ(15 downto 8) <= (others => 'Z');
  GPIO_0      <= (others => 'Z');
  GPIO_1      <= (others => 'Z');

end datapath;
