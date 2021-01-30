-- (c)EMARD
-- License=BSD

library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use ieee.numeric_std.all;

-- package for usb joystick report decoded structure
use work.report_decoded_pack.all;
-- package for LCD initialization
use work.st7789_init_pack.all;

library ecp5u;
use ecp5u.components.all;

entity ulx3s_v20_apple2 is
generic
(
  C_system_clock_hz: natural := 14375000; -- Hz 14.375 MHz for 60 Hz frame rate
  --C_system_clock_hz: natural := 13500000; -- Hz 13.5 MHz for 50 Hz frame rate
  -- enable none or one of oled options:
  C_oled_hex    : boolean := false; -- OLED display HEX debug
  C_oled_vga    : boolean := true; -- LCD ST7789 as display
  -- PS/2 keyboard at (enable one of):
  C_kbd_us2     : boolean := true;  -- onboard micro USB with OTG adapter
  C_kbd_us3     : boolean := false; -- PMOD US3 at GP,GN 25,22,21
  C_kbd_us4     : boolean := false; -- PMOD US4 at GP,GN 24,23,20
  C_kbd_esp32   : boolean := false; -- ESP32->PS2 (wifi_gpio16=clk, wifi_gpio17=data)
  -- USB joystick at (enable one of):
  C_joy_us2     : boolean := false; -- onboard micro USB with OTG adapter
  C_joy_us3     : boolean := false; -- PMOD US3 at GP,GN 25,22,21
  C_joy_us4     : boolean := false; -- PMOD US4 at GP,GN 24,23,20
  -- apple ][ disk
  C_apple2_disk : boolean := true;  -- false BTNs debug to select track
  -- C_apple2_disk = true, then enable one of
  C_sdcard      : boolean := false; -- NIB images written to raw SD card
  C_esp32       : boolean := true   -- ESP32 disk2.py micropython DISK ][ server
);
port
(
  clk_25mhz: in std_logic;  -- main clock input from 25MHz clock source

  -- UART0 (FTDI USB slave serial)
  ftdi_rxd: out   std_logic;
  ftdi_txd: in    std_logic;
  -- FTDI additional signaling
  ftdi_ndtr: inout  std_logic;
  ftdi_ndsr: inout  std_logic;
  ftdi_nrts: inout  std_logic;
  ftdi_txden: inout std_logic;

  -- UART1 (WiFi serial)
  wifi_rxd: out   std_logic;
  wifi_txd: in    std_logic;
  -- WiFi additional signaling
  wifi_en: inout  std_logic; -- '0' will disable wifi by default
  wifi_gpio0: inout std_logic;
  wifi_gpio5: inout std_logic;
  wifi_gpio16: inout std_logic;
  wifi_gpio17: inout std_logic;

  -- Onboard blinky
  led: out std_logic_vector(7 downto 0);
  btn: in std_logic_vector(6 downto 0);
  sw: in std_logic_vector(0 to 3);
  oled_csn, oled_clk, oled_mosi, oled_dc, oled_resn: out std_logic;

  -- Audio
  audio_l, audio_r, audio_v: out std_logic_vector(3 downto 0);

  -- GPIO (some are shared with wifi and adc)
  gp, gn: inout std_logic_vector(27 downto 0);

  -- FPGA direct USB connector
  usb_fpga_dp, usb_fpga_dn: in std_logic;
  usb_fpga_bd_dp, usb_fpga_bd_dn: inout std_logic;
  usb_fpga_pu_dp, usb_fpga_pu_dn: out std_logic;

  -- SHUTDOWN: logic '1' here will shutdown power on PCB >= v1.7.5
  shutdown: out std_logic := '0';

  -- Digital Video (fake differential outputs)
  gpdi_dp: out std_logic_vector(3 downto 0);

  -- Flash ROM
  --flash_miso   : in      std_logic;
  --flash_mosi   : out     std_logic;
  --flash_clk    : out     std_logic;
  --flash_csn    : out     std_logic;

  -- SD card
  sd_d: inout std_logic_vector(3 downto 0);
  sd_clk, sd_cmd: inout std_logic
);
end;

architecture Behavioral of ulx3s_v20_apple2 is
  signal S_oled: std_logic_vector(63 downto 0);
  signal S_enable: std_logic;

  signal ddr_d: std_logic_vector(3 downto 0);
  signal dvid_red, dvid_green, dvid_blue, dvid_clock: std_logic_vector(1 downto 0);
  signal clk_pixel, clk_pixel_shift: std_logic;
  signal clocks, clocks_usb: std_logic_vector(3 downto 0);
  signal locked: std_logic;

  -- APPLE ][

  -- VGA output
  signal
  VGA_CLK,                                            -- Clock
  VGA_HS,                                             -- H_SYNC
  VGA_VS,                                             -- V_SYNC
  VGA_NBLANK,                                         -- not BLANK
  VGA_BLANK,                                          -- BLANK
  VGA_SYNC : std_logic;                               -- SYNC
  signal
  VGA_R,                                              -- Red[9:0]
  VGA_G,                                              -- Green[9:0]
  VGA_B : unsigned(9 downto 0);                       -- Blue[9:0]

  -- invert active LOW sync
  signal vga_hsync, vga_vsync: std_logic;
  -- after OSD module 
  signal osd_vga_r, osd_vga_g, osd_vga_b: std_logic_vector(7 downto 0);
  signal osd_vga_hsync, osd_vga_vsync, osd_vga_blank: std_logic;
  -- SPI BTN/DISK interface
  signal spi_irq, spi_csn, spi_miso, spi_mosi, spi_sck: std_logic;
  signal spi_ram_wr, spi_ram_rd: std_logic;
  signal spi_ram_wr_data: std_logic_vector(7 downto 0);
  signal spi_ram_addr: std_logic_vector(31 downto 0); -- MSB for ROMs
  signal R_cpu_control: std_logic_vector(7 downto 0);
  signal R_btn_joy: std_logic_vector(btn'range);

  signal clk_140M, clk_28M, clk_14M, clk_2M, PRE_PHASE_ZERO: std_logic;
  signal IO_SELECT, DEVICE_SELECT : std_logic_vector(7 downto 0);
  signal ADDR : unsigned(15 downto 0);
  signal D, PD : unsigned(7 downto 0);

  signal ram_we : std_logic;
  signal VIDEO, HBL, VBL, LD194 : std_logic;
  signal COLOR_LINE : std_logic;
  signal COLOR_LINE_CONTROL : std_logic;
  signal PDL_STROBE : std_logic;
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
  signal D1_ACTIVE, D2_ACTIVE : std_logic;
  signal track_addr : unsigned(13 downto 0);
  signal TRACK_RAM_ADDR : unsigned(13 downto 0);
  signal tra : unsigned(15 downto 0);
  signal TRACK_RAM_DI : unsigned(7 downto 0);
  signal TRACK_RAM_WE : std_logic;

  signal BRAM_WE : std_logic;
  signal BRAM_DQ : std_logic_vector(7 downto 0);
  signal BRAM_ADDR : unsigned(15 downto 0);
  
  -- USB
  signal clk_usb, clk_6M: std_logic;
  signal S_hid_report: std_logic_vector(63 downto 0);
  signal S_hid_report_valid: std_logic;
  signal S_hid_report_decoded: T_report_decoded;

  -- PMOD with US3 and US4
  -- ULX3S direct or pins down and flat cable: don't swap GP/GN, normal differential input
  -- ULX3S pins up and flat cable: swap GP/GN and invert differential input

  alias us3_fpga_bd_dp: std_logic is gp(25);
  alias us3_fpga_bd_dn: std_logic is gn(25);

  alias us3_fpga_pu_dp: std_logic is gp(22);
  alias us3_fpga_pu_dn: std_logic is gn(22);

  --alias us3_fpga_n_dp: std_logic is gp(21); -- pins up flat cable
  --signal us3_fpga_dp: std_logic; -- pins up flat cable
  alias us3_fpga_dp: std_logic is gp(21); -- direct

  alias us4_fpga_bd_dp: std_logic is gp(24);
  alias us4_fpga_bd_dn: std_logic is gn(24);

  alias us4_fpga_pu_dp: std_logic is gp(23);
  alias us4_fpga_pu_dn: std_logic is gn(23);

  --alias us4_fpga_n_dp: std_logic is gp(20); -- pins up flat cable
  --signal us4_fpga_dp: std_logic; -- pins up flat cable
  alias us4_fpga_dp: std_logic is gp(20); -- direct

begin
  clk_apple2: entity work.ecp5pll
  generic map
  (
      in_hz => 25*1000000,
    out0_hz => C_system_clock_hz*10,
    out1_hz => C_system_clock_hz*2,
    out2_hz => C_system_clock_hz
  )
  port map
  (
    clk_i   => clk_25mhz,
    clk_o   => clocks,
    locked  => locked
  );
  clk_140M  <= clocks(0);
  clk_28M   <= clocks(1);
  clk_14M   <= clocks(2);

  clk_usb: entity work.ecp5pll
  generic map
  (
      in_hz =>  25*1000000,
    out0_hz => 120*1000000,
    out1_hz =>   6*1000000
  )
  port map
  (
    clk_i  => clk_25mhz,
    clk_o  => clocks_usb
  );
  clk_6M   <= clocks_usb(1);

  wifi_rxd <= ftdi_txd;
  ftdi_rxd <= wifi_txd;

  -- ESP32 -> FPGA
  spi_csn <= not wifi_gpio5;
  spi_sck <= gn(11); -- wifi_gpio25
  spi_mosi <= gp(11); -- wifi_gpio26
  -- FPGA -> ESP32
  wifi_gpio16 <= spi_miso;
  wifi_gpio0 <= not spi_irq; -- wifi_gpio0 IRQ active low

  S_enable <= not btn(1); -- BTN1 to hold OLED display

  G_oled: if C_oled_hex generate
  oled_inst: entity oled_hex_decoder
  generic map
  (
    C_data_len => S_oled'length
  )
  port map
  (
    clk => clk_14M,
    clken => flash_clk(0),
    en => S_enable,
    data => S_oled,
    spi_resn => oled_resn,
    spi_clk => oled_clk,
    spi_csn => oled_csn,
    spi_dc => oled_dc,
    spi_mosi => oled_mosi
  );
  end generate;
  
  -- APPLE ][ --

  power_on : process(CLK_14M)
  begin
    if rising_edge(CLK_14M) then
      reset <= (not R_btn_joy(0)) or (not locked);
    end if;
  end process;

  -- In the Apple ][, this was a 555 timer
  flash_clkgen : process (CLK_14M)
  begin
    if rising_edge(CLK_14M) then
      flash_clk <= flash_clk + 1;
    end if;
  end process;

  -- Paddle buttons
 
  COLOR_LINE_CONTROL <= COLOR_LINE and SW(3);  -- Color or B&W mode

  core : entity work.apple2
  port map
  (
    CLK_14M        => CLK_14M,
    CLK_2M         => CLK_2M,
    PRE_PHASE_ZERO => PRE_PHASE_ZERO,
    FLASH_CLK      => flash_clk(22),
    reset          => reset,
    ADDR           => ADDR,
    ram_addr       => BRAM_ADDR(15 downto 0),
    D              => D,
    ram_do         => unsigned(BRAM_DQ(7 downto 0)),
    PD             => PD,
    ram_we         => BRAM_WE,
    VIDEO          => VIDEO,
    COLOR_LINE     => COLOR_LINE,
    HBL            => HBL,
    VBL            => VBL,
    LD194          => LD194,
    K              => K,
    read_key       => read_key,
    AN             => led(7 downto 4),
    PDL_STROBE     => PDL_STROBE,
    GAMEPORT       => GAMEPORT,
    IO_SELECT      => IO_SELECT,
    DEVICE_SELECT  => DEVICE_SELECT,
    pcDebugOut     => cpu_pc,
    speaker        => speaker
    );
  audio_l(0) <= speaker;
  audio_l(3 downto 1) <= (others => '0');
  audio_r(0) <= speaker;
  audio_r(3 downto 1) <= (others => '0');
  audio_v(3 downto 0) <= (others => '0');

  clk_usb <= clk_6M; -- 48MHz full speed, 6MHz low speed

  G_us2: if C_joy_us2 generate
  usb_fpga_pu_dp <= '0';
  usb_fpga_pu_dn <= '0';
  us2_hid_host_inst: entity usbh_host_hid
  generic map
  (
    C_report_length_strict => '1',
    C_usb_speed => '0' -- '0':Low-speed '1':Full-speed
  )
  port map
  (
    clk => clk_usb, -- 6 MHz for low-speed USB1.0 device or 48 MHz for full-speed USB1.1 device
    bus_reset => '0',
    usb_dif => usb_fpga_dp,
    usb_dp  => usb_fpga_bd_dp,
    usb_dn  => usb_fpga_bd_dn,
    hid_report => S_hid_report,
    hid_valid => S_hid_report_valid
  );
  end generate;
  
  G_us3: if C_joy_us3 generate
  us3_fpga_pu_dp <= '0';
  us3_fpga_pu_dn <= '0';
  --us3_fpga_dp <= not us3_fpga_n_dp; -- pins up flat cable
  us3_fpga_dp <= 'Z'; -- pins down flat cable
  us3_hid_host_inst: entity usbh_host_hid
  generic map
  (
    C_report_length_strict => '1',
    C_usb_speed => '0' -- '0':Low-speed '1':Full-speed
  )
  port map
  (
    clk => clk_usb, -- 6 MHz for low-speed USB1.0 device or 48 MHz for full-speed USB1.1 device
    bus_reset => '0',
    usb_dif => us3_fpga_dp,
    usb_dp  => us3_fpga_bd_dp,
    usb_dn  => us3_fpga_bd_dn,
    hid_report => S_hid_report,
    hid_valid => S_hid_report_valid
  );
  end generate;

  G_us4: if C_joy_us4 generate
  us4_fpga_pu_dp <= '0';
  us4_fpga_pu_dn <= '0';
  --us4_fpga_dp <= not us4_fpga_n_dp; -- pins up flat cable
  us4_fpga_dp <= 'Z'; -- pins down flat cable
  us4_hid_host_inst: entity usbh_host_hid
  generic map
  (
    C_report_length_strict => '1',
    C_usb_speed => '0' -- '0':Low-speed '1':Full-speed
  )
  port map
  (
    clk => clk_usb, -- 6 MHz for low-speed USB1.0 device or 48 MHz for full-speed USB1.1 device
    bus_reset => '0',
    usb_dif => us4_fpga_dp,
    usb_dp  => us4_fpga_bd_dp,
    usb_dn  => us4_fpga_bd_dn,
    hid_report => S_hid_report,
    hid_valid => S_hid_report_valid
  );
  end generate;

  usbhid_report_decoder: entity usbhid_report_decoder
  port map
  (
    clk => clk_usb,
    hid_report => S_hid_report,
    hid_valid => S_hid_report_valid,
    decoded => S_hid_report_decoded
  );

  paddle : entity work.paddle
  port map(
    CLK_14M    => CLK_14M,
    PDL0       => S_hid_report_decoded.lstick_x,
    PDL1       => S_hid_report_decoded.lstick_y,
    PDL2       => S_hid_report_decoded.rstick_x,
    PDL3       => S_hid_report_decoded.rstick_y,
    PDL_STROBE => PDL_STROBE,
    GAMEPORT   => GAMEPORT(7 downto 4)
  );
  GAMEPORT(3 downto 0) <=
  (
    (S_hid_report_decoded.btn_rbumper  or S_hid_report_decoded.btn_start) &
    (S_hid_report_decoded.btn_ltrigger or S_hid_report_decoded.btn_b or S_hid_report_decoded.btn_x) &
    (S_hid_report_decoded.btn_rtrigger or S_hid_report_decoded.btn_a or S_hid_report_decoded.btn_y)
  ) & "0"; -- last 0 is for cassette (not used, we have floppy)

  vga : entity work.vga_controller port map (
    CLK_28M    => CLK_28M,
    VIDEO      => VIDEO,
    COLOR_LINE => COLOR_LINE_CONTROL,
    HBL        => HBL,
    VBL        => VBL,
    LD194      => LD194,
    VGA_CLK    => VGA_CLK,
    VGA_HS     => VGA_HS,
    VGA_VS     => VGA_VS,
    VGA_BLANK  => VGA_NBLANK,
    VGA_R      => VGA_R,
    VGA_G      => VGA_G,
    VGA_B      => VGA_B
    );
  VGA_SYNC <= '0';

  G_kbd_us2: if C_kbd_us2 generate
  keyboard : entity work.keyboard port map (
    PS2_Clk  => usb_fpga_bd_dp,
    PS2_Data => usb_fpga_bd_dn,
    CLK_14M  => CLK_14M,
    reset    => reset,
    read     => read_key,
    K        => K
    );
  usb_fpga_pu_dp <= '1';
  usb_fpga_pu_dn <= '1';
  end generate;

  G_kbd_us3: if C_kbd_us3 generate
  keyboard : entity work.keyboard port map (
    PS2_Clk  => us3_fpga_bd_dp,
    PS2_Data => us3_fpga_bd_dn,
    CLK_14M  => CLK_14M,
    reset    => reset,
    read     => read_key,
    K        => K
    );
  us3_fpga_pu_dp <= '1';
  us3_fpga_pu_dn <= '1';
  end generate;

  G_kbd_us4: if C_kbd_us4 generate
  keyboard : entity work.keyboard port map (
    PS2_Clk  => us4_fpga_bd_dp,
    PS2_Data => us4_fpga_bd_dn,
    CLK_14M  => CLK_14M,
    reset    => reset,
    read     => read_key,
    K        => K
    );
  us4_fpga_pu_dp <= '1';
  us4_fpga_pu_dn <= '1';
  end generate;

  G_kbd_esp32: if C_kbd_esp32 generate
  keyboard : entity work.keyboard port map (
    PS2_Clk  => gp(11), -- wifi_gpio26,
    PS2_Data => gn(11), -- wifi_gpio25,
    CLK_14M  => CLK_14M,
    reset    => reset,
    read     => read_key,
    K        => K
    );
  end generate;

  G_yes_apple2_disk: if C_apple2_disk generate
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
  led(0) <= D1_ACTIVE;
  led(1) <= D2_ACTIVE;
  end generate; -- apple2_disk
  
  G_not_apple2_disk: if not C_apple2_disk generate
  TRACK(4 downto 0) <= btn(6 downto 2);
  end generate; -- not apple2_disk, tracks selected by BTNs

  G_apple2_sdcard: if C_sdcard generate
  sdcard_interface : entity work.spi_controller port map (
    CLK_14M        => CLK_14M,
    RESET          => RESET,

    CS_N           => sd_d(3),
    SCLK           => sd_clk,
    MOSI           => sd_cmd,
    MISO           => sd_d(0),

    SDHC           => led(2),

    track          => TRACK,
    image          => image,

    ram_write_addr => TRACK_RAM_ADDR,
    ram_di         => TRACK_RAM_DI,
    ram_we         => TRACK_RAM_WE
    );
  sd_d(1) <= 'Z';
  sd_d(2) <= 'Z';
  S_oled(45 downto 32) <= TRACK_RAM_ADDR;
  S_oled(55 downto 48) <= TRACK_RAM_DI;
  -- selects disk image
  --image <= "0000000" & SW(2 downto 0);
  image <= "000000" & x"0";
  end generate; -- apple2_sdcard

  process(CLK_14M)
  begin
    if rising_edge(CLK_14M) then
      R_btn_joy <= btn;
    end if;
  end process;

  G_disk2_spi_slave: if C_esp32 generate
  B_disk2_spi_slave: block
    signal spi_rd, spi_wr: std_logic;
    signal spi_addr: std_logic_vector(31 downto 0);
    signal spi_data_out, spi_data_in: std_logic_vector(7 downto 0);
    signal R_track : unsigned(track'range);
    signal R_track_irq: std_logic;
  begin
  process(CLK_14M)
  begin
    if rising_edge(CLK_14M) then
      R_track <= track;
      if R_track /= track then
        R_track_irq <= '1';
      else
        R_track_irq <= '0';
      end if;
    end if;
  end process;

  E_disk2_spi_ram_btn: entity work.spi_ram_btn
  generic map
  (
    c_sclk_capable_pin => 0,
    c_addr_bits => 32
  )
  port map
  (
    clk => CLK_14M,
    csn => spi_csn,
    sclk => spi_sck,
    mosi => spi_mosi,
    miso => spi_miso,
    btn => R_btn_joy,
    irq => spi_irq,
    floppy_req_type => std_logic_vector("00" & track),
    floppy_req => R_track_irq,
    wr => spi_wr,
    rd => spi_rd,
    addr => spi_addr,
    data_in => spi_data_in,
    data_out => spi_data_out
  );

  TRACK_RAM_WE <= '1' when spi_wr = '1' and spi_addr(31 downto 30) = "00" else '0'; -- write disk track to 0x0000
  TRACK_RAM_ADDR <= unsigned(spi_addr(TRACK_RAM_ADDR'range));
  TRACK_RAM_DI <= unsigned(spi_data_out);
  -- read: 0x0000 track and irq state, 0xFE00 btn state
  --spi_data_in <= std_logic_vector("00" & TRACK) when spi_addr(15 downto 14) = "00" else "0" & btn;
  --spi_data_in <= std_logic_vector(R_btn_irq & R_track_irq & TRACK) when spi_addr(15 downto 14) = "00" else "0" & btn;
  sd_d(3) <= 'Z';
  sd_d(1) <= 'Z';
  sd_d(2) <= 'Z';
  S_oled(45 downto 32) <= TRACK_RAM_ADDR; -- disk writes to track buffer
  --S_oled(45 downto 32) <= TRACK_ADDR; -- CPU reads from track buffer
  S_oled(55 downto 48) <= TRACK_RAM_DI;

  end block;
  end generate; -- disk2_spi_slave

  ram_64K: entity work.bram_true2p_1clk
  generic map
  (
        dual_port => true,
        pass_thru_a => false,
        data_width => 8,
        addr_width => 16
  )
  port map
  (
        clk => CLK_14M,
        we_a => bram_we,
        addr_a => std_logic_vector(bram_addr),
        data_in_a => std_logic_vector(d),
        data_out_a => bram_dq,
        we_b => btn(2), -- press BTN2 to spoil CRC of warmstart reset vector
        addr_b => x"03F4", -- reset vector CRC address 0x03F4
        data_in_b => std_logic_vector(flash_clk(7 downto 0)) -- random
  );

  -- Processor PC on the right four digits
  S_oled(15 downto 0) <= cpu_pc;

  -- Current disk track on middle two digits 
  S_oled(21 downto 16) <= track;

  -- Current disk image on left two digits
  S_oled(31 downto 24) <= image(7 downto 0);
  
  clk_pixel <= CLK_28M;
  clk_pixel_shift <= CLK_140M;

  vga_blank <= not vga_nblank;
  vga_hsync <= not vga_hs;
  vga_vsync <= not vga_vs;

  -- SPI OSD pipeline
  spi_osd_inst: entity work.spi_osd
  generic map
  (
    c_start_x => 26, c_start_y => 32, -- xy centered
    c_char_bits_x => 6, c_chars_y => 20, -- xy size, slightly less than full screen
    c_init_on => 0, -- 1:OSD initially shown without any SPI init
    c_char_file => "osd.mem", -- initial OSD content
    c_font_file => "font_bizcat8x16.mem"
  )
  port map
  (
    clk_pixel => clk_pixel, clk_pixel_ena => '1',
    i_r => std_logic_vector(vga_r(9 downto 2)),
    i_g => std_logic_vector(vga_g(9 downto 2)),
    i_b => std_logic_vector(vga_b(9 downto 2)),
    i_hsync => vga_hsync, i_vsync => vga_vsync, i_blank => vga_blank,
    i_csn => spi_csn, i_sclk => spi_sck, i_mosi => spi_mosi,
    o_r => osd_vga_r, o_g => osd_vga_g, o_b => osd_vga_b,
    o_hsync => osd_vga_hsync, o_vsync => osd_vga_vsync, o_blank => osd_vga_blank
  );

  vga2dvi_converter: entity work.vga2dvid
  generic map
  (
      C_ddr     => '1',
      C_depth   => 8 -- 8bpp (8 bit per pixel)
  )
  port map
  (
      clk_pixel => clk_pixel, -- 28 MHz
      clk_shift => clk_pixel_shift, -- 5*28 MHz

      in_red   => osd_vga_r,
      in_green => osd_vga_g,
      in_blue  => osd_vga_b,

      in_hsync => osd_vga_hsync,
      in_vsync => osd_vga_vsync,
      in_blank => osd_vga_blank,

      -- single-ended output ready for differential buffers
      out_red   => dvid_red,
      out_green => dvid_green,
      out_blue  => dvid_blue,
      out_clock => dvid_clock
  );

  -- vendor specific DDR modules
  -- convert SDR 2-bit input to DDR clocked 1-bit output (single-ended)
  ddr_clock: ODDRX1F port map (D0=>dvid_clock(0), D1=>dvid_clock(1), Q=>gpdi_dp(3), SCLK=>clk_pixel_shift, RST=>'0');
  ddr_red:   ODDRX1F port map (D0=>dvid_red(0),   D1=>dvid_red(1),   Q=>gpdi_dp(2), SCLK=>clk_pixel_shift, RST=>'0');
  ddr_green: ODDRX1F port map (D0=>dvid_green(0), D1=>dvid_green(1), Q=>gpdi_dp(1), SCLK=>clk_pixel_shift, RST=>'0');
  ddr_blue:  ODDRX1F port map (D0=>dvid_blue(0),  D1=>dvid_blue(1),  Q=>gpdi_dp(0), SCLK=>clk_pixel_shift, RST=>'0');

  yes_oled_vga: if C_oled_vga generate
  oled_vga_blk: block
    constant c_offset_x : natural :=  67; -- x-centering inc->move picture left
    constant c_offset_y : natural :=   2; -- y-centering inc->move picture up
    constant c_size_x   : natural := 240; -- 280 native
    constant c_size_y   : natural := 240; -- 192 native
    signal hsyncedge: std_logic_vector(1 downto 0);
    signal big_blank: std_logic;
    signal lcd_line_ena, lcd_pixel_ena: std_logic;
    signal s_vga_lcd_pixel: std_logic_vector(15 downto 0);
    signal s_custom_blankn, s_custom_blank: std_logic;
  begin
  -- every 2nd line, every 2nd pixel
  process(clk_pixel)
  begin
    if rising_edge(clk_pixel) then
      hsyncedge <= vga_hsync & hsyncedge(1);
      if hsyncedge = "10" then
        lcd_line_ena <= not lcd_line_ena;
      end if;
      lcd_pixel_ena <= not lcd_pixel_ena;
      if vga_hsync = '1' or vga_vsync = '1' then
        big_blank <= '1';
      else
        big_blank <= '0';
      end if;
    end if;
  end process;
  -- generate custom blank to center xy
  lcd_custom_blank_inst: entity work.osd_vhd
  generic map
  (
    c_x_start       => c_offset_x*2, -- *2 to match doublescan
    c_x_stop        => c_offset_x*2+c_size_x*2+9,
    c_y_start       => c_offset_y*2,
    c_y_stop        => c_offset_y*2+c_size_y*2+2,
    c_x_bits        => 11, -- bits in x counter
    c_y_bits        => 10, -- bits in y counter
    c_transparency  =>  0  -- 0 for custom blank
  )
  port map
  (
    clk_pixel => clk_pixel,
    clk_pixel_ena => '1',
    i_r => x"00", i_g => x"00", i_b => x"00",
    i_hsync => vga_hsync,
    i_vsync => vga_vsync,
    i_blank => big_blank,
    i_osd_en => '0',
    i_osd_r => x"00", i_osd_g => x"00", i_osd_b => x"00",
    o_osd_en => s_custom_blankn
  );
  s_custom_blank <= (not lcd_line_ena) or (not s_custom_blankn);
  s_vga_lcd_pixel(15 downto 11) <= std_logic_vector(osd_vga_r(7 downto 3));
  s_vga_lcd_pixel(10 downto  5) <= std_logic_vector(osd_vga_g(7 downto 2));
  s_vga_lcd_pixel( 4 downto  0) <= std_logic_vector(osd_vga_b(7 downto 3));
  lcd_vga_inst: entity work.spi_display
  generic map
  (
    c_clk_spi_mhz  => C_system_clock_hz*10/1000000,
    c_reset_us     => 1,
    c_color_bits   => 16,
    c_clk_phase    => '0',
    c_clk_polarity => '1',
    c_init_seq     => c_st7789_init_seq,
    c_nop          => x"00"
  )
  port map
  (
    reset          => reset,
    clk_pixel      => clk_pixel, -- 28 MHz
    clk_pixel_ena  => lcd_pixel_ena,
    clk_spi        => clk_pixel_shift, -- 140 MHz
    clk_spi_ena    => '1',
    vsync          => vga_vsync,
    blank          => s_custom_blank,
    color          => s_vga_lcd_pixel,
    spi_resn       => oled_resn,
    spi_clk        => oled_clk, -- clk/2 (display max 64 MHz, overclock 70 MHz acceptable)
    --spi_csn        => oled_csn, -- for 8-pin 1.54" ST7789
    spi_dc         => oled_dc,
    spi_mosi       => oled_mosi
  );
  oled_csn <= '1'; -- for 7-pin 1.3" ST7789
  end block;
  end generate;

end Behavioral;
