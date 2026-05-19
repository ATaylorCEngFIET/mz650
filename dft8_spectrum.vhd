library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.fixed_pkg.all;

-- 8-point Discrete Fourier Transform spectrum analyser, DSP48-targeted.

entity dft8_spectrum is
  generic (
    WIDTH : positive := 16
  );
  port (
    clk          : in  std_logic;
    rst          : in  std_logic;
    sample_valid : in  std_logic;
    sample_i     : in  sfixed(WIDTH - 1 downto 0);
    output_valid : out std_logic;
    bin_index_o  : out ufixed(2 downto 0);
    magnitude_o  : out ufixed(31 downto 0)
  );
end entity dft8_spectrum;

architecture rtl of dft8_spectrum is

  -- 12-bit signed twiddle constants (max magnitude 1024 fits in 11 bits + sign).
  constant TW_WIDTH : positive := 12;
  type tw_array_t is array (0 to 7) of signed(TW_WIDTH - 1 downto 0);

  constant TW_COS : tw_array_t := (
    to_signed( 1024, TW_WIDTH),
    to_signed(  724, TW_WIDTH),
    to_signed(    0, TW_WIDTH),
    to_signed( -724, TW_WIDTH),
    to_signed(-1024, TW_WIDTH),
    to_signed( -724, TW_WIDTH),
    to_signed(    0, TW_WIDTH),
    to_signed(  724, TW_WIDTH)
  );

  -- Pre-negated sin twiddles so the imag MAC chain is a pure accumulate.
  -- TW_NSIN(i) = -sin(2*pi*i/8) scaled by 1024.
  constant TW_NSIN : tw_array_t := (
    to_signed(    0, TW_WIDTH),
    to_signed( -724, TW_WIDTH),
    to_signed(-1024, TW_WIDTH),
    to_signed( -724, TW_WIDTH),
    to_signed(    0, TW_WIDTH),
    to_signed(  724, TW_WIDTH),
    to_signed( 1024, TW_WIDTH),
    to_signed(  724, TW_WIDTH)
  );

  -- Sample buffer typed as signed so the multiplier port widths stay inside
  -- the DSP48 envelope. 
  type sample_array_t is array (0 to 7) of signed(WIDTH - 1 downto 0);
  signal samples_r      : sample_array_t := (others => (others => '0'));
  signal sample_count_r : natural range 0 to 8 := 0;
  signal frame_ready_r  : std_logic := '0';

  -- Datapath widths
  constant PROD_WIDTH : positive := WIDTH + TW_WIDTH;   -- 16 + 12 = 28
  constant ACC_WIDTH  : positive := PROD_WIDTH + 4;     -- 32, with headroom for 8 products
  constant MAG_WIDTH  : positive := 32;

  -- Control FSM
  type state_t is (S_IDLE, S_COMPUTE, S_CAPTURE_RESULT, S_EMIT);
  signal state_r     : state_t := S_IDLE;
  signal bin_cycle_r : natural range 0 to 10 := 0;
  signal k_r         : natural range 0 to 3  := 0;

  -- MAC pipeline stage 1: operand registers (DSP48 A/B regs).
  signal sample_a_r : signed(WIDTH    - 1 downto 0) := (others => '0');
  signal cos_b_r    : signed(TW_WIDTH - 1 downto 0) := (others => '0');
  signal nsin_b_r   : signed(TW_WIDTH - 1 downto 0) := (others => '0');
  signal first_s1_r : std_logic := '0';

  -- MAC pipeline stage 2: product registers (DSP48 M reg).
  signal prod_cos_r : signed(PROD_WIDTH - 1 downto 0) := (others => '0');
  signal prod_sin_r : signed(PROD_WIDTH - 1 downto 0) := (others => '0');
  signal first_s2_r : std_logic := '0';

  -- MAC pipeline stage 3: accumulators (DSP48 P reg).
  signal acc_re_r : signed(ACC_WIDTH - 1 downto 0) := (others => '0');
  signal acc_im_r : signed(ACC_WIDTH - 1 downto 0) := (others => '0');

  attribute use_dsp : string;
  attribute use_dsp of acc_re_r : signal is "yes";
  attribute use_dsp of acc_im_r : signal is "yes";

  -- Per-bin magnitude store.
  type mag_array_t is array (0 to 3) of unsigned(MAG_WIDTH - 1 downto 0);
  signal mags_r : mag_array_t := (others => (others => '0'));

  -- Output emitter.
  signal emit_idx_r  : natural range 0 to 4 := 4;
  signal out_valid_r : std_logic := '0';
  signal bin_r       : ufixed(2 downto 0)  := (others => '0');
  signal mag_r       : ufixed(31 downto 0) := (others => '0');

  constant MAG_MAX : natural := 2 ** 30;

begin

  ------------------------------------------------------------------
  -- Sample capture: load 8 samples then assert frame_ready_r once.
  ------------------------------------------------------------------
  capture : process(clk)
  begin
    if rising_edge(clk) then
      frame_ready_r <= '0';
      if rst = '1' then
        sample_count_r <= 0;
        for i in samples_r'range loop
          samples_r(i) <= (others => '0');
        end loop;
      elsif sample_valid = '1' and state_r = S_IDLE then
        samples_r(sample_count_r) <= signed(to_slv(sample_i));
        if sample_count_r = 7 then
          sample_count_r <= 0;
          frame_ready_r  <= '1';
        else
          sample_count_r <= sample_count_r + 1;
        end if;
      end if;
    end if;
  end process;

  ------------------------------------------------------------------
  -- Control + emit FSM.
  --
  --   S_IDLE              : wait for frame_ready_r.
  --   S_COMPUTE           : drive 8 operands into the MAC pipeline, then
  --                         wait 2 drain cycles for the pipeline to settle.
  --                         bin_cycle_r counts 0..10 per bin.
  --   S_CAPTURE_RESULT    : capture |acc_re|/1024 + |acc_im|/1024 into
  --                         mags_r(k_r). Advance to next bin or to emit.
  --   S_EMIT              : stream four (bin, magnitude) results out.
  ------------------------------------------------------------------
  control : process(clk)
    variable tw_idx_v : natural range 0 to 7;
    variable re_abs_v : unsigned(ACC_WIDTH - 1 downto 0);
    variable im_abs_v : unsigned(ACC_WIDTH - 1 downto 0);
    variable mag_v    : natural;
  begin
    if rising_edge(clk) then
      out_valid_r <= '0';

      if rst = '1' then
        state_r     <= S_IDLE;
        bin_cycle_r <= 0;
        k_r         <= 0;
        emit_idx_r  <= 4;
        sample_a_r  <= (others => '0');
        cos_b_r     <= (others => '0');
        nsin_b_r    <= (others => '0');
        first_s1_r  <= '0';
        bin_r       <= (others => '0');
        mag_r       <= (others => '0');
        mags_r      <= (others => (others => '0'));
      else

        case state_r is

          when S_IDLE =>
            sample_a_r <= (others => '0');
            cos_b_r    <= (others => '0');
            nsin_b_r   <= (others => '0');
            first_s1_r <= '0';
            if frame_ready_r = '1' then
              state_r     <= S_COMPUTE;
              bin_cycle_r <= 0;
              k_r         <= 0;
            end if;

          when S_COMPUTE =>
            if bin_cycle_r <= 7 then
              tw_idx_v   := (k_r * bin_cycle_r) mod 8;
              sample_a_r <= samples_r(bin_cycle_r);
              cos_b_r    <= TW_COS(tw_idx_v);
              nsin_b_r   <= TW_NSIN(tw_idx_v);
              if bin_cycle_r = 0 then
                first_s1_r <= '1';
              else
                first_s1_r <= '0';
              end if;
            else
              -- Drain: zero operands so spurious products do not enter the acc.
              sample_a_r <= (others => '0');
              cos_b_r    <= (others => '0');
              nsin_b_r   <= (others => '0');
              first_s1_r <= '0';
            end if;

            if bin_cycle_r = 10 then
              state_r     <= S_CAPTURE_RESULT;
              bin_cycle_r <= 0;
            else
              bin_cycle_r <= bin_cycle_r + 1;
            end if;

          when S_CAPTURE_RESULT =>
            -- Pipeline drained; acc_re_r and acc_im_r hold the full bin sums.
            -- Match the reference magnitude scaling: |Re|/1024 + |Im|/1024.
            if acc_re_r(acc_re_r'high) = '1' then
              re_abs_v := unsigned(-acc_re_r);
            else
              re_abs_v := unsigned(acc_re_r);
            end if;
            if acc_im_r(acc_im_r'high) = '1' then
              im_abs_v := unsigned(-acc_im_r);
            else
              im_abs_v := unsigned(acc_im_r);
            end if;
            mags_r(k_r) <= shift_right(re_abs_v, 10) + shift_right(im_abs_v, 10);

            sample_a_r <= (others => '0');
            cos_b_r    <= (others => '0');
            nsin_b_r   <= (others => '0');
            first_s1_r <= '0';

            if k_r = 3 then
              state_r    <= S_EMIT;
              emit_idx_r <= 0;
            else
              k_r         <= k_r + 1;
              state_r     <= S_COMPUTE;
              bin_cycle_r <= 0;
            end if;

          when S_EMIT =>
            sample_a_r <= (others => '0');
            cos_b_r    <= (others => '0');
            nsin_b_r   <= (others => '0');
            first_s1_r <= '0';
            if emit_idx_r < 4 then
              out_valid_r <= '1';
              bin_r <= to_ufixed(emit_idx_r, bin_r'high, bin_r'low);
              if mags_r(emit_idx_r) > to_unsigned(MAG_MAX, MAG_WIDTH) then
                mag_v := MAG_MAX;
              else
                mag_v := to_integer(mags_r(emit_idx_r));
              end if;
              mag_r <= to_ufixed(mag_v, mag_r'high, mag_r'low);
              emit_idx_r <= emit_idx_r + 1;
            else
              state_r <= S_IDLE;
            end if;

        end case;
      end if;
    end if;
  end process;

  ------------------------------------------------------------------
  -- MAC pipeline. Two independent MAC chains feeding two DSP48 slices.
  -- Pattern: A/B operand reg (in control process) -> M reg (prod_*_r)
  -- -> P reg (acc_*_r). This is the canonical inference pattern from
  -- AMD UG901 / UG479.
  ------------------------------------------------------------------
  mac : process(clk)
  begin
    if rising_edge(clk) then
      if rst = '1' then
        prod_cos_r <= (others => '0');
        prod_sin_r <= (others => '0');
        first_s2_r <= '0';
        acc_re_r   <= (others => '0');
        acc_im_r   <= (others => '0');
      else
        -- Stage 2 (M register): multiply
        prod_cos_r <= sample_a_r * cos_b_r;
        prod_sin_r <= sample_a_r * nsin_b_r;
        first_s2_r <= first_s1_r;

        -- Stage 3 (P register): accumulate, with load-on-first behaviour
        if first_s2_r = '1' then
          acc_re_r <= resize(prod_cos_r, ACC_WIDTH);
          acc_im_r <= resize(prod_sin_r, ACC_WIDTH);
        else
          acc_re_r <= acc_re_r + resize(prod_cos_r, ACC_WIDTH);
          acc_im_r <= acc_im_r + resize(prod_sin_r, ACC_WIDTH);
        end if;
      end if;
    end if;
  end process;

  output_valid <= out_valid_r;
  bin_index_o  <= bin_r;
  magnitude_o  <= mag_r;

end architecture rtl;
