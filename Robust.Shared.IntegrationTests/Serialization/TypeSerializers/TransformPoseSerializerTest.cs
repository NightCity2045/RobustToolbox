using System.Globalization;
using System.Numerics;
using NUnit.Framework;
using Robust.Shared.Maths;
using Robust.Shared.Serialization;
using Robust.Shared.Serialization.Manager;
using Robust.Shared.Serialization.Manager.Attributes;
using Robust.Shared.Serialization.Markdown.Mapping;
using Robust.Shared.Serialization.Markdown.Validation;
using Robust.Shared.Serialization.Markdown.Value;
using Robust.Shared.Serialization.TypeSerializers.Implementations.Custom;

namespace Robust.UnitTesting.Shared.Serialization.TypeSerializers;

[TestFixture]
public sealed partial class TransformPoseSerializerTest : OurSerializationTest
{
    [TestCase("-12.5,45.25")]
    [TestCase("-12.5 x 45.25")]
    public void LegacyPositionImportsAtZeroHeight(string text)
    {
        var node = new ValueDataNode(text);
        var position = Serialization.Read<Vector3, ValueDataNode, TransformPositionSerializer>(node);
        Assert.That(position, Is.EqualTo(new Vector3(-12.5f, 45.25f, 0)));
        var written = (ValueDataNode) Serialization.WriteValue<Vector3, TransformPositionSerializer>(position);
        Assert.That(written.Value, Is.EqualTo("-12.5,45.25,0"));
    }

    [TestCase("90")]
    [TestCase("1.5707963267948966 rad")]
    public void LegacyAngleRotatesAroundPositiveZ(string text)
    {
        var rotation = Serialization.Read<Quaternion, ValueDataNode, TransformRotationSerializer>(new ValueDataNode(text));
        Assert.That(Vector3.Distance(Vector3.Transform(Vector3.UnitX, rotation), Vector3.UnitY), Is.LessThan(1e-6));
        Assert.That(Vector3.Distance(Vector3.Transform(Vector3.UnitZ, rotation), Vector3.UnitZ), Is.LessThan(1e-6));
    }

    [Test]
    public void CanonicalPoseRoundTripsPitchRollAndNegativeHeightAcrossCultures()
    {
        var original = new PoseDefinition
        {
            Position = new Vector3(-100000.25f, 65536.5f, -11.75f),
            Rotation = new RigidTransform3(Vector3.Zero,
                Quaternion.CreateFromYawPitchRoll(0.43f, -1.2f, 2.3f)).Rotation,
        };
        var culture = CultureInfo.CurrentCulture;
        try
        {
            CultureInfo.CurrentCulture = CultureInfo.GetCultureInfo("ru-RU");
            var mapping = Serialization.WriteValueAs<MappingDataNode>(original, alwaysWrite: true);
            CultureInfo.CurrentCulture = CultureInfo.GetCultureInfo("en-US");
            var restored = Serialization.Read<PoseDefinition>(mapping, notNullableOverride: true);
            Assert.That(restored.Position, Is.EqualTo(original.Position));
            foreach (var axis in new[] { Vector3.UnitX, Vector3.UnitY, Vector3.UnitZ })
                Assert.That(Vector3.Distance(Vector3.Transform(axis, original.Rotation),
                    Vector3.Transform(axis, restored.Rotation)), Is.LessThan(1e-6));
            Assert.That(mapping.Cast<ValueDataNode>("pos").Value, Is.EqualTo("-100000.25,65536.5,-11.75"));
            Assert.That(mapping.Cast<ValueDataNode>("rot").Value.Split(','), Has.Length.EqualTo(4));
        }
        finally
        {
            CultureInfo.CurrentCulture = culture;
        }
    }

    [TestCase("NaN,0,0")]
    [TestCase("0,Infinity,0")]
    [TestCase("0,0,-Infinity")]
    [TestCase("1e100,0")]
    [TestCase("1,2,3,4")]
    [TestCase("1,,3")]
    public void InvalidPositionIsRejectedByValidatorAndReader(string text)
    {
        var node = new ValueDataNode(text);
        Assert.That(Serialization.ValidateNode<Vector3, ValueDataNode, TransformPositionSerializer>(node), Is.TypeOf<ErrorNode>());
        Assert.Throws<InvalidMappingException>(() => Serialization.Read<Vector3, ValueDataNode, TransformPositionSerializer>(node));
    }

    [TestCase("0,0,0,0")]
    [TestCase("NaN,0,0,1")]
    [TestCase("0,Infinity,0,1")]
    [TestCase("1,2,3")]
    [TestCase("NaN rad")]
    [TestCase("Infinity")]
    [TestCase("1e400")]
    public void InvalidRotationIsRejectedByValidatorAndReader(string text)
    {
        var node = new ValueDataNode(text);
        Assert.That(Serialization.ValidateNode<Quaternion, ValueDataNode, TransformRotationSerializer>(node), Is.TypeOf<ErrorNode>());
        Assert.Throws<InvalidMappingException>(() => Serialization.Read<Quaternion, ValueDataNode, TransformRotationSerializer>(node));
    }

    [TestCase("0,0,0,-3e38")]
    [TestCase("0,0,0,-1e-38")]
    public void ScaledQuaternionHasOneCanonicalRepresentation(string text)
    {
        var rotation = Serialization.Read<Quaternion, ValueDataNode, TransformRotationSerializer>(new ValueDataNode(text));
        Assert.That(rotation, Is.EqualTo(Quaternion.Identity));
        var written = (ValueDataNode) Serialization.WriteValue<Quaternion, TransformRotationSerializer>(rotation);
        Assert.That(written.Value, Is.EqualTo("0,0,0,1"));
        Assert.That(Serialization.Read<Quaternion, ValueDataNode, TransformRotationSerializer>(written), Is.EqualTo(rotation));
    }

    [TestCase("1e308")]
    [TestCase("-1e308 rad")]
    public void LargeFiniteLegacyAnglesNormalizeWithoutOverflow(string text)
    {
        var node = new ValueDataNode(text);
        Assert.That(Serialization.ValidateNode<Quaternion, ValueDataNode, TransformRotationSerializer>(node), Is.TypeOf<ValidatedValueNode>());
        var rotation = Serialization.Read<Quaternion, ValueDataNode, TransformRotationSerializer>(node);
        Assert.That(new RigidTransform3(Vector3.Zero, rotation).IsValid, Is.True);
    }

    [Test]
    public void WritersRejectInvalidState()
    {
        Assert.Throws<ArgumentOutOfRangeException>(() => Serialization.WriteValue<Vector3, TransformPositionSerializer>(new Vector3(0, 0, float.NaN)));
        Assert.Throws<ArgumentOutOfRangeException>(() => Serialization.WriteValue<Quaternion, TransformRotationSerializer>(default));
    }

    [DataDefinition]
    private sealed partial class PoseDefinition
    {
        [DataField("pos", customTypeSerializer: typeof(TransformPositionSerializer))]
        public Vector3 Position;

        [DataField("rot", customTypeSerializer: typeof(TransformRotationSerializer))]
        public Quaternion Rotation = Quaternion.Identity;
    }
}
