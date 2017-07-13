/**
  * \author Sergio Agostinho - sergio.r.agostinho@gmail.com
  * \date created: 2017/05/05
  * \date last modified: 2017/05/05
  */
#include <cvl/io/grabber.h>
#include <gtest/gtest.h>

using namespace ht;

class GrabberFixture : public ::testing::Test
{
  public:

    class GrabberBare : public Grabber
    {
      public:

        GrabberBare ( const std::set<const char*>& supported_sigs,
                      const std::set<Mode>& supported_modes = std::set<Mode> {Mode::ASYNC})
          : Grabber (supported_sigs, supported_modes)
        {}

        bool start () override { return true; }

        void stop () override {}
    };

    GrabberFixture ()
      : sig_void_ (typeid (void(void)).name ())
      , sig_complex_ (typeid (int(char, float)).name ())
      , signatures_ {sig_void_, sig_complex_}
      , grabber_ (signatures_)
    {}

  protected:

    const char* const sig_void_;

    const char* const sig_complex_;

    const std::set<const char*> signatures_;

    GrabberBare grabber_;
};


TEST_F (GrabberFixture, constructor) {

  const std::set<const char*>& grabber_sigs  = grabber_.getSupportedSigs ();

  // tests
  ASSERT_EQ (2u, grabber_sigs.size ());

  std::set<const char*>::const_iterator it = grabber_sigs.find (sig_void_);
  ASSERT_NE (it, grabber_sigs.cend ());

  it = grabber_sigs.find (sig_complex_);
  ASSERT_NE (it, grabber_sigs.cend ());
}

TEST_F (GrabberFixture, isSupported) {

  // tests
  ASSERT_TRUE (grabber_.isSupported<void (void)> ());
  ASSERT_TRUE (grabber_.isSupported<int (char, float)> ());
  ASSERT_FALSE (grabber_.isSupported<float (float)> ());
}
