--======================================================================================--
--          Verilog to VHDL conversion by Martin Neumann martin@neumnns-mail.de         --
--                                                                                      --
--          ///////////////////////////////////////////////////////////////////         --
--          //                                                               //         --
--          //  USB 1.1 PHY                                                  //         --
--          //                                                               //         --
--          //                                                               //         --
--          //  Author: Rudolf Usselmann                                     //         --
--          //          rudi@asics.ws                                        //         --
--          //                                                               //         --
--          //                                                               //         --
--          //  Downloaded from: http://www.opencores.org/cores/usb_phy/     //         --
--          //                                                               //         --
--          ///////////////////////////////////////////////////////////////////         --
--          //                                                               //         --
--          //  Copyright (C) 2000-2002 Rudolf Usselmann                     //         --
--          //                          www.asics.ws                         //         --
--          //                          rudi@asics.ws                        //         --
--          //                                                               //         --
--          //  This source file may be used and distributed without         //         --
--          //  restriction provided that this copyright statement is not    //         --
--          //  removed from the file and that any derivative work contains  //         --
--          //  the original copyright notice and the associated disclaimer. //         --
--          //                                                               //         --
--          //      THIS SOFTWARE IS PROVIDED ``AS IS'' AND WITHOUT ANY      //         --
--          //  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED    //         --
--          //  TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS    //         --
--          //  FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL THE AUTHOR       //         --
--          //  OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,          //         --
--          //  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES     //         --
--          //  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE    //         --
--          //  GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR         //         --
--          //  BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF   //         --
--          //  LIABILITY, WHETHER IN  CONTRACT, STRICT LIABILITY, OR TORT   //         --
--          //  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT   //         --
--          //  OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE          //         --
--          //  POSSIBILITY OF SUCH DAMAGE.                                  //         --
--          //                                                               //         --
--          ///////////////////////////////////////////////////////////////////         --
--======================================================================================--
 
library ieee;
use ieee.std_logic_1164.all;
use ieee.std_logic_arith.all;
use ieee.std_logic_unsigned.all;
 
entity usb_phy is
  generic (
    usb_rst_det : boolean := true
  );
  port (
    clk              : in  std_logic;  -- 48 MHz
    rst              : in  std_logic;
    phy_tx_mode      : in  std_logic;  -- HIGH level for differential io mode (else single-ended)
    usb_rst          : out std_logic;
    -- Transciever Interface
    rxd, rxdp, rxdn  : in  std_logic;
    txdp, txdn, txoe : out std_logic;
    -- clk recovery debug
    ce_o             : out std_logic; 
    -- RX debug interface
    sync_err_o, bit_stuff_err_o, byte_err_o: out std_logic;
    -- UTMI Interface
    LineCtrl_i       : IN  STD_LOGIC := '0'; -- 0: normal data TX, 1: line control (for low speed usb)
    DataOut_i        : in  std_logic_vector(7 downto 0); -- byte_to_send or LineCtrl mode (see usb_tx_phy)
    TxValid_i        : in  std_logic;
    TxReady_o        : out std_logic;
    DataIn_o         : out std_logic_vector(7 downto 0);
    RxValid_o        : out std_logic;
    RxActive_o       : out std_logic;
    RxError_o        : out std_logic;
    LineState_o      : out std_logic_vector(1 downto 0)
  );
end usb_phy;
 
architecture RTL of usb_phy is
 
--component usb_tx_phy
--port (
--  clk              : in  std_logic;
--  rst              : in  std_logic;
--  fs_ce            : in  std_logic;
--  phy_mode         : in  std_logic;
--  -- Transciever Interface
--  txdp, txdn, txoe : out std_logic;
--  -- UTMI Interface
--  DataOut_i        : in  std_logic_vector(7 downto 0);
--  TxValid_i        : in  std_logic;
--  TxReady_o        : out std_logic
--);
--end component;
--
--component usb_rx_phy
--port (
--  clk              : in  std_logic;
--  rst              : in  std_logic;
--  -- Transciever Interface
--  fs_ce            : out std_logic;
--  rxd, rxdp, rxdn  : in  std_logic;
--  -- UTMI Interface
--  DataIn_o         : out std_logic_vector(7 downto 0);
--  RxValid_o        : out std_logic;
--  RxActive_o       : out std_logic;
--  RxError_o        : out std_logic;
--  RxEn_i           : in  std_logic;
--  LineState        : out std_logic_vector(1 downto 0)
--);
--end component;
 
  signal LineState      : std_logic_vector(1 downto 0);
  signal fs_ce          : std_logic;
  signal rst_cnt        : std_logic_vector(4 downto 0);
  signal txoe_out       : std_logic;
  signal usb_rst_out    : std_logic := '0';
  
begin
 
--======================================================================================--
  -- Misc Logic                                                                         --
--======================================================================================--
 
  usb_rst      <= usb_rst_out;
  LineState_o  <= LineState;
  txoe         <= txoe_out;
 
--======================================================================================--
  -- TX Phy                                                                             --
--======================================================================================--
 
  i_tx_phy: entity work.usb_tx_phy
  port map (
    clk        => clk,
    rst        => rst,
    fs_ce      => fs_ce,
    phy_mode   => phy_tx_mode,
    -- Transciever Interface
    txdp       => txdp,
    txdn       => txdn,
    txoe       => txoe_out,
    -- UTMI Interface
    LineCtrl_i => LineCtrl_i,
    DataOut_i  => DataOut_i,
    TxValid_i  => TxValid_i,
    TxReady_o  => TxReady_o
  );
 
--======================================================================================--
  -- RX Phy and DPLL                                                                    --
--======================================================================================--
  E_rx_phy_emard: entity work.usb_rx_phy
  port map (
    clk        => clk,
    reset      => not rst,
    clk_recovered_edge => fs_ce,
    -- Transciever Interface
    usb_dif    => rxd,
    usb_dp     => rxdp,
    usb_dn     => rxdn,
    -- UTMI Interface
    data       => DataIn_o,
    valid      => RxValid_o,
    rx_active  => RxActive_o,
    rx_en      => txoe_out,
    linestate  => LineState
  );
  RxError_o <= '0';
  ce_o <= fs_ce;
 
--======================================================================================--
  -- Generate an USB Reset if we see SE0 for at least 2.5uS                             --
--======================================================================================--
 
  usb_rst_g : if usb_rst_det generate
    p_rst_cnt: process (clk, rst)
    begin
      if rst ='0' then
        rst_cnt <= (others => '0');
      elsif rising_edge(clk) then
        if LineState /= "00" then
          rst_cnt <= (others => '0');
        elsif usb_rst_out ='0' and fs_ce ='1' then
          rst_cnt <= rst_cnt + 1;
        end if;
      end if;
    end process;
 
    p_usb_rst_out: process (clk)
    begin
      if rising_edge(clk) then
        if rst_cnt = "11111" then
          usb_rst_out  <= '1';
        else
          usb_rst_out  <= '0';
        end if;
      end if;
    end process;
  end generate;
 
end RTL;
